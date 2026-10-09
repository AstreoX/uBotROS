#include "UBotROSSensorBridgeComponent.h"

#include "Components/SceneComponent.h"
#include "GameFramework/Actor.h"
#include "HAL/PlatformTime.h"
#include "UBotClock.h"
#include "UBotIdentityComponent.h"
#include "UBotOdometrySensorComponent.h"
#include "UBotROS.h"
#include "UBotROSBridgeSubsystem.h"
#include "UBotROSConverters.h"
#include "UBotROSMessageTypes.h"
#include "UBotROSSensorPublisher.h"
#include "UBotRosConventions.h"
#include "UBotSensorInterface.h"
#include "UBotUnits.h"
#include "UObject/WeakObjectPtr.h"

namespace UBotROSSensorBridge
{
    /** How long the bridge may stay disconnected while the component is Ready before one Warning is logged. */
    constexpr double DisconnectedWarningDelaySeconds = 2.0;

    /** Interval of the connection monitor tick. */
    constexpr float MonitorTickIntervalSeconds = 0.5f;

    const TCHAR* const DefaultOdometryFrame = TEXT("odom");
    const TCHAR* const DefaultBaseFrame = TEXT("base_link");

    FString DescribeComponent(const UObject* Object)
    {
        return Object ? Object->GetName() : FString(TEXT("<none>"));
    }
}

UUBotROSSensorBridgeComponent::UUBotROSSensorBridgeComponent()
{
    // The tick only runs the connection monitor while the component is Ready.
    PrimaryComponentTick.bCanEverTick = true;
    PrimaryComponentTick.bStartWithTickEnabled = false;
    PrimaryComponentTick.TickInterval = UBotROSSensorBridge::MonitorTickIntervalSeconds;
}

void UUBotROSSensorBridgeComponent::BeginPlay()
{
    Super::BeginPlay();
    if (bAutoInitialize)
    {
        InitializeBridge();
    }
}

void UUBotROSSensorBridgeComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    StopROSBridge();
    Super::EndPlay(EndPlayReason);
}

TArray<FString> UUBotROSSensorBridgeComponent::GetActivePublisherTopics() const
{
    return ActivePublisherTopics;
}

FString UUBotROSSensorBridgeComponent::GetResolvedNamespace() const
{
    return ResolvedNamespace.IsEmpty() ? ComputeNamespace() : ResolvedNamespace;
}

FString UUBotROSSensorBridgeComponent::GetResolvedBaseFrameId() const
{
    return ResolvedBaseFrameId.IsEmpty() ? ResolveFrameId(GetRawBaseFrameId()) : ResolvedBaseFrameId;
}

bool UUBotROSSensorBridgeComponent::RestartROSBridge()
{
    StopROSBridge();
    return InitializeBridge();
}

void UUBotROSSensorBridgeComponent::StopROSBridge()
{
    StopConnectionMonitor();
    TearDownPublishers();
    Bridge.Reset();
    ResolvedNamespace.Reset();
    ResolvedBaseFrameId.Reset();
    ResolvedOdometryFrameId.Reset();
    if (Status != EUBotROSSensorBridgeStatus::Uninitialized)
    {
        Status = EUBotROSSensorBridgeStatus::Stopped;
    }
}

bool UUBotROSSensorBridgeComponent::InitializeBridge()
{
    if (Status == EUBotROSSensorBridgeStatus::Initializing)
    {
        return false;
    }

    Status = EUBotROSSensorBridgeStatus::Initializing;
    LastError.Reset();

    UUBotROSBridgeSubsystem* Subsystem = UUBotROSBridgeSubsystem::Get(this);
    if (!Subsystem)
    {
        SetError(TEXT("The uBot ROS bridge subsystem is not available: UBotROS publishes from game and PIE worlds only."));
        return false;
    }
    Bridge = Subsystem;

    // Tear down before reporting, so an OnROSBridgeError handler may restart the bridge safely.
    FString ConfigurationError;
    if (!ConfigurePublishers(ConfigurationError))
    {
        TearDownPublishers();
        Bridge.Reset();
        SetError(ConfigurationError);
        return false;
    }

    PublishStaticTransforms();
    SetReady();
    UE_LOG(LogUBotROS, Log, TEXT("ROS sensor bridge '%s' on '%s' ready: namespace '/%s', %d sensor(s), %d topic(s)."),
        *GetName(), *UBotROSSensorBridge::DescribeComponent(GetOwner()), *ResolvedNamespace, DiscoveredSensors.Num(), ActivePublisherTopics.Num());
    OnROSBridgeReady.Broadcast();
    return true;
}

bool UUBotROSSensorBridgeComponent::RefreshSensorPublishers()
{
    if (!Bridge.IsValid())
    {
        return InitializeBridge();
    }
    if (Status == EUBotROSSensorBridgeStatus::Initializing)
    {
        return false;
    }

    StopConnectionMonitor();
    TearDownPublishers();
    LastError.Reset();
    Status = EUBotROSSensorBridgeStatus::Initializing;

    FString ConfigurationError;
    if (!ConfigurePublishers(ConfigurationError))
    {
        // The bridge reference is kept so that a later refresh retries the configuration.
        TearDownPublishers();
        SetError(ConfigurationError);
        return false;
    }

    PublishStaticTransforms();
    SetReady();
    OnROSPublishersRefreshed.Broadcast(ActivePublisherTopics.Num());
    return true;
}

bool UUBotROSSensorBridgeComponent::ConfigurePublishers(FString& OutError)
{
    AActor* Owner = GetOwner();
    UUBotROSBridgeSubsystem* Subsystem = Bridge.Get();
    if (!Owner || !Subsystem)
    {
        OutError = TEXT("ROS sensor bridge has no valid owner actor or uBot ROS bridge subsystem.");
        return false;
    }

    if (!ResolveIdentity(OutError))
    {
        return false;
    }

    TArray<UObject*> FoundSensors;
    UBot::Sensors::FindSensors(Owner, FoundSensors);

    struct FCandidate
    {
        UObject* Sensor = nullptr;
        TSharedPtr<IUBotROSSensorPublisherFactory> Factory;
        FName FactoryName;
    };

    TArray<FCandidate> Candidates;
    TMap<FName, int32> SensorsPerFactory;
    for (UObject* Sensor : FoundSensors)
    {
        TSharedPtr<IUBotROSSensorPublisherFactory> Factory = UBot::ROSPublishing::FindFactory(Sensor);
        if (!Factory.IsValid())
        {
            UE_LOG(LogUBotROS, Verbose, TEXT("No ROS publisher factory supports sensor '%s' on '%s'; it is not published."),
                *Sensor->GetName(), *Owner->GetName());
            continue;
        }
        FCandidate& Candidate = Candidates.AddDefaulted_GetRef();
        Candidate.Sensor = Sensor;
        Candidate.Factory = Factory;
        Candidate.FactoryName = Factory->GetExtensionName();
        ++SensorsPerFactory.FindOrAdd(Candidate.FactoryName);
    }

    // Legacy validation rules. Sensor ids and frame ids are compared case-insensitively like the legacy
    // (lower-cased) names, because override keys are case-insensitive too.
    TMap<FName, TSet<FString>> SensorTokensPerFactory;
    TSet<FString> SensorFrameIds;
    for (const FCandidate& Candidate : Candidates)
    {
        const FString RawSensorId = IUBotSensor::Execute_GetSensorId(Candidate.Sensor).TrimStartAndEnd();
        const FString FrameId = ResolveFrameId(IUBotSensor::Execute_GetSensorFrameId(Candidate.Sensor));
        if (RawSensorId.IsEmpty() || FrameId.IsEmpty())
        {
            OutError = FString::Printf(TEXT("Sensor '%s' requires non-empty SensorId and FrameId for ROS publishing."), *Candidate.Sensor->GetName());
            return false;
        }

        const FString SensorToken = UBot::Ros::SanitizeRosName(RawSensorId);
        TSet<FString>& Tokens = SensorTokensPerFactory.FindOrAdd(Candidate.FactoryName);
        if (Tokens.Contains(SensorToken))
        {
            OutError = FString::Printf(TEXT("Duplicate SensorId '%s' exists within the same sensor type."), *SensorToken);
            return false;
        }
        Tokens.Add(SensorToken);

        if (SensorFrameIds.Contains(FrameId))
        {
            OutError = FString::Printf(TEXT("Duplicate sensor FrameId '%s' cannot be used for static TF."), *FrameId);
            return false;
        }
        SensorFrameIds.Add(FrameId);
    }

    const FUBotROSQos SensorQos = ResolveQos(SensorDataQosOverride, FUBotROSQos::SensorData());
    const FUBotROSQos StateQos = ResolveQos(StateQosOverride, FUBotROSQos::State());
    const TWeakObjectPtr<UUBotROSSensorBridgeComponent> WeakThis(this);

    for (const FCandidate& Candidate : Candidates)
    {
        const int32 PublisherIndex = Publishers.Num();

        FUBotROSPublisherContext Context;
        Context.Bridge = Subsystem;
        Context.Namespace = ResolvedNamespace;
        Context.SensorQos = SensorQos;
        Context.StateQos = StateQos;
        Context.bPublishInvalidSamples = bPublishInvalidSamples;
        Context.bSkipWhenNoSubscribers = bSkipWhenNoSubscribers;
        Context.SensorsOfSameType = SensorsPerFactory.FindRef(Candidate.FactoryName);
        Context.ResolveTopic = [WeakThis, PublisherIndex](const FString& LogicalKey, const FString& DefaultRelativePath) -> FString
        {
            UUBotROSSensorBridgeComponent* Self = WeakThis.Get();
            return Self ? Self->ResolveTopicForPublisher(LogicalKey, DefaultRelativePath, PublisherIndex) : FString();
        };
        Context.ResolveFrameId = [WeakThis](const FString& RawFrameId) -> FString
        {
            const UUBotROSSensorBridgeComponent* Self = WeakThis.Get();
            return Self ? Self->ResolveFrameId(RawFrameId) : UBot::ROSPublishing::SanitizeRosPath(RawFrameId);
        };

        UUBotROSSensorPublisher* Publisher = Candidate.Factory->CreatePublisher(this, Candidate.Sensor);
        if (!Publisher)
        {
            OutError = FString::Printf(TEXT("ROS publisher factory '%s' did not create a publisher for sensor '%s'."),
                *Candidate.FactoryName.ToString(), *Candidate.Sensor->GetName());
            return false;
        }

        PendingTopicError.Reset();
        FString PublisherError;
        if (!Publisher->Initialize(Candidate.Sensor, Context, PublisherError))
        {
            Publisher->Shutdown();
            if (!PendingTopicError.IsEmpty())
            {
                OutError = PendingTopicError;
            }
            else if (!PublisherError.IsEmpty())
            {
                OutError = PublisherError;
            }
            else
            {
                OutError = FString::Printf(TEXT("Failed to create the ROS publisher for sensor '%s'."), *Candidate.Sensor->GetName());
            }
            return false;
        }

        if (!ClaimPublisherTopics(*Publisher, PublisherIndex))
        {
            Publisher->Shutdown();
            OutError = PendingTopicError;
            return false;
        }

        Publishers.Add(Publisher);
        DiscoveredSensors.Add(Candidate.Sensor);
        ActivePublisherTopics.Append(Publisher->GetTopics());
    }

    ActivePublisherTopics.Sort();
    ConfigureDynamicTransform();
    return true;
}

void UUBotROSSensorBridgeComponent::TearDownPublishers()
{
    if (UUBotOdometrySensorComponent* OdometrySensor = DynamicTFOdometrySensor.Get())
    {
        OdometrySensor->OnOdometryDataUpdated.RemoveDynamic(this, &UUBotROSSensorBridgeComponent::HandleOdometryTransform);
    }
    DynamicTFOdometrySensor.Reset();

    for (UUBotROSSensorPublisher* Publisher : Publishers)
    {
        if (Publisher)
        {
            Publisher->Shutdown();
        }
    }
    Publishers.Reset();
    DiscoveredSensors.Reset();
    ActivePublisherTopics.Reset();
    ClaimedTopics.Reset();
    PendingTopicError.Reset();

    if (UUBotROSBridgeSubsystem* Subsystem = Bridge.Get())
    {
        Subsystem->ClearStaticTransforms(this);
    }
}

void UUBotROSSensorBridgeComponent::ConfigureDynamicTransform()
{
    DynamicTFOdometrySensor.Reset();
    if (!bPublishDynamicTF)
    {
        return;
    }

    UUBotOdometrySensorComponent* OdometrySensor = nullptr;
    for (UObject* Sensor : DiscoveredSensors)
    {
        OdometrySensor = Cast<UUBotOdometrySensorComponent>(Sensor);
        if (OdometrySensor)
        {
            break;
        }
    }
    if (!OdometrySensor)
    {
        return;
    }

    USceneComponent* BaseComponent = ResolveBaseComponent();
    const FString ExpectedOdometryFrame = UBot::ROSPublishing::SanitizeRosPath(GetRawOdometryFrameId());
    const FString ExpectedBaseFrame = UBot::ROSPublishing::SanitizeRosPath(GetRawBaseFrameId());
    const FString SensorOdometryFrame = UBot::ROSPublishing::SanitizeRosPath(OdometrySensor->OdometryFrameId);
    const FString SensorChildFrame = UBot::ROSPublishing::SanitizeRosPath(OdometrySensor->ChildFrameId);
    const bool bValid = BaseComponent
        && OdometrySensor->GetTrackedComponent() == BaseComponent
        && SensorOdometryFrame.Equals(ExpectedOdometryFrame, ESearchCase::CaseSensitive)
        && SensorChildFrame.Equals(ExpectedBaseFrame, ESearchCase::CaseSensitive);
    if (!bValid)
    {
        UE_LOG(LogUBotROS, Error, TEXT("Dynamic TF disabled: odometry sensor '%s' tracks '%s' with frames '%s' -> '%s', but the ROS sensor bridge on '%s' expects base component '%s' with frames '%s' -> '%s'."),
            *OdometrySensor->GetName(), *UBotROSSensorBridge::DescribeComponent(OdometrySensor->GetTrackedComponent()), *SensorOdometryFrame, *SensorChildFrame,
            *UBotROSSensorBridge::DescribeComponent(GetOwner()), *UBotROSSensorBridge::DescribeComponent(BaseComponent), *ExpectedOdometryFrame, *ExpectedBaseFrame);
        return;
    }

    DynamicTFOdometrySensor = OdometrySensor;
    OdometrySensor->OnOdometryDataUpdated.AddUniqueDynamic(this, &UUBotROSSensorBridgeComponent::HandleOdometryTransform);
}

void UUBotROSSensorBridgeComponent::PublishStaticTransforms()
{
    UUBotROSBridgeSubsystem* Subsystem = Bridge.Get();
    if (!Subsystem)
    {
        return;
    }
    if (!bPublishStaticSensorTF)
    {
        Subsystem->ClearStaticTransforms(this);
        return;
    }

    USceneComponent* BaseComponent = ResolveBaseComponent();
    if (!BaseComponent)
    {
        UE_LOG(LogUBotROS, Error, TEXT("Static sensor TF publishing skipped because the base link component of '%s' is invalid."),
            *UBotROSSensorBridge::DescribeComponent(GetOwner()));
        Subsystem->ClearStaticTransforms(this);
        return;
    }

    // Physical offsets: scale is removed from both transforms so a scaled actor does not scale its TF.
    FTransform BaseWorld = BaseComponent->GetComponentTransform();
    BaseWorld.SetScale3D(FVector::OneVector);
    const double StampSeconds = UBot::Clock::GetSimTimeSeconds(this);

    TArray<FUBotRosTransformStamped> Transforms;
    for (const UUBotROSSensorPublisher* Publisher : Publishers)
    {
        UObject* Sensor = Publisher ? Publisher->GetSensor() : nullptr;
        if (!Sensor)
        {
            continue;
        }

        const FString& SensorFrameId = Publisher->GetSensorFrameId();
        USceneComponent* FrameComponent = Sensor->GetClass()->ImplementsInterface(UUBotSensor::StaticClass())
            ? IUBotSensor::Execute_GetSensorFrameComponent(Sensor)
            : nullptr;
        if (!Publisher->PublishesPhysicalFrame())
        {
            // The odometry sensor's FrameId ("odom" by default) is the odometry frame, the parent of the base
            // frame. A static base -> odom transform would close a loop with the dynamic odom -> base
            // transform and fight with map -> odom of SLAM / AMCL.
            UE_LOG(LogUBotROS, Verbose, TEXT("Sensor '%s' does not publish a physical frame; no static TF %s -> %s."), *Publisher->GetSensorId(), *ResolvedBaseFrameId, *SensorFrameId);
        }
        else if (!FrameComponent)
        {
            UE_LOG(LogUBotROS, Verbose, TEXT("Sensor '%s' has no frame component; no static TF %s -> %s."), *Publisher->GetSensorId(), *ResolvedBaseFrameId, *SensorFrameId);
        }
        else if (SensorFrameId.Equals(ResolvedBaseFrameId, ESearchCase::CaseSensitive))
        {
            UE_LOG(LogUBotROS, Verbose, TEXT("Sensor '%s' publishes in the base frame '%s'; no static TF needed."), *Publisher->GetSensorId(), *SensorFrameId);
        }
        else if (SensorFrameId.Equals(ResolvedOdometryFrameId, ESearchCase::CaseSensitive))
        {
            // Any sensor named like the odometry frame would make the odometry frame a child of the base frame.
            UE_LOG(LogUBotROS, Warning, TEXT("Sensor '%s' uses the odometry frame '%s' as its frame; no static TF %s -> %s is published (it would loop with the dynamic %s -> %s transform). Give the sensor its own FrameId."),
                *Publisher->GetSensorId(), *SensorFrameId, *ResolvedBaseFrameId, *SensorFrameId, *ResolvedOdometryFrameId, *ResolvedBaseFrameId);
        }
        else
        {
            FTransform SensorWorld = FrameComponent->GetComponentTransform();
            SensorWorld.SetScale3D(FVector::OneVector);
            Transforms.Add(UBot::ROSConvert::MakeTransform(SensorWorld.GetRelativeTransform(BaseWorld), StampSeconds, ResolvedBaseFrameId, SensorFrameId));
        }

        if (Publisher->UsesOpticalFrame())
        {
            Transforms.Add(UBot::ROSConvert::MakeOpticalTransform(StampSeconds, SensorFrameId, Publisher->GetOpticalFrameId()));
        }
    }

    Subsystem->SetStaticTransforms(this, Transforms);
}

void UUBotROSSensorBridgeComponent::HandleOdometryTransform(const FUBotOdometrySensorData& Data)
{
    if (!IsROSBridgeReady() || !bPublishDynamicTF || !(Data.Header.bValid || bPublishInvalidSamples))
    {
        return;
    }
    UUBotROSBridgeSubsystem* Subsystem = Bridge.Get();
    if (!Subsystem)
    {
        return;
    }

    // The odometry pose is the base expressed in the odometry frame (UE axes, meters).
    const FTransform OdometryFromBase(Data.OrientationQuaternion, UBot::Units::MetersToCentimeters(Data.PositionMeters), FVector::OneVector);
    TArray<FUBotRosTransformStamped> Transforms;
    Transforms.Add(UBot::ROSConvert::MakeTransform(OdometryFromBase, Data.Header.TimestampSeconds, ResolvedOdometryFrameId, ResolvedBaseFrameId));
    Subsystem->PublishTransforms(Transforms);
}

void UUBotROSSensorBridgeComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

    UUBotROSBridgeSubsystem* Subsystem = Bridge.Get();
    if (Status != EUBotROSSensorBridgeStatus::Ready || !Subsystem)
    {
        StopConnectionMonitor();
        return;
    }

    const double NowSeconds = FPlatformTime::Seconds();
    if (Subsystem->IsBridgeConnected())
    {
        if (bWarnedDisconnected)
        {
            UE_LOG(LogUBotROS, Log, TEXT("ROS sensor bridge on '%s': the uBot ROS bridge is connected again."), *UBotROSSensorBridge::DescribeComponent(GetOwner()));
        }
        bWarnedDisconnected = false;
        DisconnectedSinceSeconds = -1.0;
        return;
    }

    if (DisconnectedSinceSeconds < 0.0)
    {
        DisconnectedSinceSeconds = NowSeconds;
        return;
    }
    if (!bWarnedDisconnected && NowSeconds - DisconnectedSinceSeconds >= UBotROSSensorBridge::DisconnectedWarningDelaySeconds)
    {
        bWarnedDisconnected = true;
        UE_LOG(LogUBotROS, Warning, TEXT("ROS sensor bridge on '%s' is ready but the uBot ROS bridge is not connected (%s). Best-effort sensor data is dropped and reliable data is queued until ubot_ros_bridge is reachable."),
            *UBotROSSensorBridge::DescribeComponent(GetOwner()), *UEnum::GetValueAsString(Subsystem->GetBridgeStatus()));
    }
}

void UUBotROSSensorBridgeComponent::SetError(const FString& ErrorMessage)
{
    StopConnectionMonitor();
    LastError = ErrorMessage;
    Status = EUBotROSSensorBridgeStatus::Error;
    UE_LOG(LogUBotROS, Error, TEXT("%s"), *ErrorMessage);
    OnROSBridgeError.Broadcast(ErrorMessage);
}

void UUBotROSSensorBridgeComponent::SetReady()
{
    Status = EUBotROSSensorBridgeStatus::Ready;
    DisconnectedSinceSeconds = -1.0;
    bWarnedDisconnected = false;
    SetComponentTickEnabled(true);
}

void UUBotROSSensorBridgeComponent::StopConnectionMonitor()
{
    SetComponentTickEnabled(false);
    DisconnectedSinceSeconds = -1.0;
    bWarnedDisconnected = false;
}

bool UUBotROSSensorBridgeComponent::ResolveIdentity(FString& OutError)
{
    // An empty namespace is the root namespace (override "/"); the identity fallback is never empty.
    ResolvedNamespace = ComputeNamespace();
    ResolvedBaseFrameId = ResolveFrameId(GetRawBaseFrameId());
    ResolvedOdometryFrameId = ResolveFrameId(GetRawOdometryFrameId());
    if (ResolvedBaseFrameId.IsEmpty() || ResolvedOdometryFrameId.IsEmpty())
    {
        OutError = FString::Printf(TEXT("ROS sensor bridge on '%s' could not resolve its base frame ('%s') or odometry frame ('%s')."),
            *UBotROSSensorBridge::DescribeComponent(GetOwner()), *GetRawBaseFrameId(), *GetRawOdometryFrameId());
        return false;
    }
    return true;
}

FString UUBotROSSensorBridgeComponent::ComputeNamespace() const
{
    // Same rules as the velocity command component: a non-blank override wins and "/" alone selects the
    // root namespace; a blank override falls back to the owner's uBot identity.
    const FString Override = RobotNamespaceOverride.TrimStartAndEnd();
    if (!Override.IsEmpty())
    {
        return UBot::ROSPublishing::SanitizeRosPath(Override);
    }
    return UBot::ROSPublishing::SanitizeRosPath(UBot::Identity::ResolveNamespace(GetOwner()));
}

FString UUBotROSSensorBridgeComponent::GetRawBaseFrameId() const
{
    if (!UBot::ROSPublishing::SanitizeRosPath(BaseFrameOverride).IsEmpty())
    {
        return BaseFrameOverride.TrimStartAndEnd();
    }
    const FString IdentityFrame = UBot::Identity::ResolveBaseFrameId(GetOwner());
    return UBot::ROSPublishing::SanitizeRosPath(IdentityFrame).IsEmpty() ? FString(UBotROSSensorBridge::DefaultBaseFrame) : IdentityFrame;
}

FString UUBotROSSensorBridgeComponent::GetRawOdometryFrameId() const
{
    return UBot::ROSPublishing::SanitizeRosPath(OdometryFrame).IsEmpty() ? FString(UBotROSSensorBridge::DefaultOdometryFrame) : OdometryFrame.TrimStartAndEnd();
}

FString UUBotROSSensorBridgeComponent::ResolveFrameId(const FString& RawFrameId) const
{
    const FString Frame = UBot::ROSPublishing::SanitizeRosPath(RawFrameId);
    if (Frame.IsEmpty() || !bPrefixFramesWithNamespace)
    {
        return Frame;
    }

    const FString Namespace = ResolvedNamespace.IsEmpty() ? ComputeNamespace() : ResolvedNamespace;
    if (Namespace.IsEmpty() || Frame.StartsWith(Namespace + TEXT("/"), ESearchCase::CaseSensitive))
    {
        return Frame;
    }
    return Namespace + TEXT("/") + Frame;
}

USceneComponent* UUBotROSSensorBridgeComponent::ResolveBaseComponent() const
{
    AActor* Owner = GetOwner();
    if (!Owner)
    {
        return nullptr;
    }
    // An empty FComponentReference resolves to the owner's root component, so "unset" must be detected
    // explicitly; otherwise the base component of the owner's uBot identity could never be used.
    const bool bConfigured = BaseLinkComponent.OverrideComponent.IsValid()
        || !BaseLinkComponent.ComponentProperty.IsNone()
        || !BaseLinkComponent.PathToComponent.IsEmpty()
        || BaseLinkComponent.OtherActor.IsValid();
    if (bConfigured)
    {
        if (USceneComponent* Explicit = Cast<USceneComponent>(BaseLinkComponent.GetComponent(Owner)))
        {
            return Explicit;
        }
    }
    // Empty, unresolved or not a scene component: the identity's base component.
    return UBot::Identity::ResolveBaseComponent(Owner);
}

FUBotROSQos UUBotROSSensorBridgeComponent::ResolveQos(const FUBotROSQosOverride& Override, const FUBotROSQos& Default) const
{
    if (!Override.bOverride)
    {
        return Default;
    }
    FUBotROSQos Qos = Override.Qos;
    Qos.Depth = FMath::Max(1, Qos.Depth);
    return Qos;
}

TArray<UUBotROSSensorPublisher*> UUBotROSSensorBridgeComponent::GetPublishers() const
{
    TArray<UUBotROSSensorPublisher*> Result;
    Result.Reserve(Publishers.Num());
    for (UUBotROSSensorPublisher* Publisher : Publishers)
    {
        Result.Add(Publisher);
    }
    return Result;
}

UUBotROSSensorPublisher* UUBotROSSensorBridgeComponent::FindPublisherForSensor(const UObject* Sensor) const
{
    if (!Sensor)
    {
        return nullptr;
    }
    for (UUBotROSSensorPublisher* Publisher : Publishers)
    {
        if (Publisher && Publisher->GetSensor() == Sensor)
        {
            return Publisher;
        }
    }
    return nullptr;
}

FString UUBotROSSensorBridgeComponent::ResolveTopicForPublisher(const FString& LogicalKey, const FString& DefaultRelativePath, int32 PublisherIndex)
{
    FString Topic;
    const FUBotROSTopicOverride* Override = TopicOverrides.Find(LogicalKey);
    if (Override && Override->bOverride)
    {
        Topic = Override->Topic.TrimStartAndEnd();
        if (!Topic.StartsWith(TEXT("/"), ESearchCase::CaseSensitive))
        {
            Topic = TEXT("/") + Topic;
        }
    }
    else
    {
        Topic = UBot::ROSPublishing::JoinTopic(ResolvedNamespace, DefaultRelativePath);
    }

    // A second topic resolved by the same publisher must not collide with its first one either.
    return ClaimTopic(Topic, PublisherIndex, false) ? Topic : FString();
}

bool UUBotROSSensorBridgeComponent::ClaimTopic(const FString& Topic, int32 PublisherIndex, bool bAcceptOwnClaim)
{
    FString Reason;
    if (!UBot::ROSPublishing::IsValidAbsoluteTopic(Topic, &Reason))
    {
        PendingTopicError = FString::Printf(TEXT("Resolved ROS topic '%s' is not a valid absolute topic: %s."), *Topic, *Reason);
        return false;
    }

    for (const FClaimedTopic& Claimed : ClaimedTopics)
    {
        if (Claimed.Topic.Equals(Topic, ESearchCase::CaseSensitive))
        {
            if (bAcceptOwnClaim && Claimed.PublisherIndex == PublisherIndex)
            {
                return true;
            }
            PendingTopicError = FString::Printf(TEXT("ROS topic conflict: '%s' is assigned more than once."), *Topic);
            return false;
        }
    }

    FClaimedTopic& NewClaim = ClaimedTopics.AddDefaulted_GetRef();
    NewClaim.Topic = Topic;
    NewClaim.PublisherIndex = PublisherIndex;
    return true;
}

bool UUBotROSSensorBridgeComponent::ClaimPublisherTopics(const UUBotROSSensorPublisher& Publisher, int32 PublisherIndex)
{
    const TArray<FString> Topics = Publisher.GetTopics();
    for (int32 Index = 0; Index < Topics.Num(); ++Index)
    {
        for (int32 Earlier = 0; Earlier < Index; ++Earlier)
        {
            if (Topics[Earlier].Equals(Topics[Index], ESearchCase::CaseSensitive))
            {
                PendingTopicError = FString::Printf(TEXT("ROS topic conflict: '%s' is assigned more than once."), *Topics[Index]);
                return false;
            }
        }
        if (!ClaimTopic(Topics[Index], PublisherIndex, true))
        {
            return false;
        }
    }
    return true;
}
