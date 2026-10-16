#include "UBotROSSensorPublisher.h"

#include "Algo/StableSort.h"
#include "UBotROS.h"
#include "UBotROSBridgeSubsystem.h"
#include "UBotROSConventionHelpers.h"
#include "UBotROSProtocol.h"
#include "UBotSensorInterface.h"
#include "UBotTypes.h"

bool UUBotROSSensorPublisher::Initialize(UObject* Sensor, const FUBotROSPublisherContext& InContext, FString& OutError)
{
    Shutdown();
    RawSensorId.Reset();
    SensorFrameId.Reset();

    if (!IsValid(Sensor))
    {
        OutError = FString::Printf(TEXT("%s: no valid sensor to publish."), *GetClass()->GetName());
        return false;
    }
    if (!IsValid(InContext.Bridge))
    {
        OutError = FString::Printf(TEXT("%s: no uBot ROS bridge subsystem for sensor '%s'."), *GetClass()->GetName(), *Sensor->GetName());
        return false;
    }

    SensorPtr = Sensor;
    BridgePtr = InContext.Bridge;
    PublisherContext = InContext;
    PublisherContext.Bridge = nullptr;

    FString RawFrameId;
    if (Sensor->GetClass()->ImplementsInterface(UUBotSensor::StaticClass()))
    {
        RawSensorId = IUBotSensor::Execute_GetSensorId(Sensor).TrimStartAndEnd();
        RawFrameId = IUBotSensor::Execute_GetSensorFrameId(Sensor).TrimStartAndEnd();
    }
    if (RawSensorId.IsEmpty())
    {
        RawSensorId = Sensor->GetName();
    }
    if (RawFrameId.IsEmpty())
    {
        RawFrameId = RawSensorId;
    }

    SensorFrameId = ResolveFrameId(RawFrameId);
    if (SensorFrameId.IsEmpty())
    {
        OutError = FString::Printf(TEXT("Sensor '%s' has no usable frame id ('%s')."), *RawSensorId, *RawFrameId);
        Shutdown();
        return false;
    }

    bPublisherInitialized = true;
    return true;
}

void UUBotROSSensorPublisher::Shutdown()
{
    if (UUBotROSBridgeSubsystem* Bridge = BridgePtr.Get())
    {
        for (const FAdvertisedTopic& Advertised : AdvertisedTopics)
        {
            Bridge->Unadvertise(Advertised.Id);
        }
    }
    AdvertisedTopics.Reset();
    bPublisherInitialized = false;
    SensorPtr.Reset();
    BridgePtr.Reset();
}

TArray<FString> UUBotROSSensorPublisher::GetTopics() const
{
    TArray<FString> Topics;
    Topics.Reserve(AdvertisedTopics.Num());
    for (const FAdvertisedTopic& Advertised : AdvertisedTopics)
    {
        Topics.Add(Advertised.Topic);
    }
    return Topics;
}

FString UUBotROSSensorPublisher::GetSensorToken() const
{
    return UBot::ROSConventions::SanitizeRosName(RawSensorId);
}

FString UUBotROSSensorPublisher::GetOpticalFrameId() const
{
    return SensorFrameId + UBot::ROSConventions::OpticalFrameSuffix();
}

int32 UUBotROSSensorPublisher::AdvertiseTopic(const FString& LogicalKey, const FString& DefaultRelativePath, const FString& MessageType, const FUBotROSQos& Qos, FString& OutError)
{
    UUBotROSBridgeSubsystem* Bridge = BridgePtr.Get();
    if (!Bridge)
    {
        OutError = FString::Printf(TEXT("Sensor '%s': the uBot ROS bridge subsystem is gone."), *RawSensorId);
        return 0;
    }

    const FString Topic = ResolveTopic(LogicalKey, DefaultRelativePath);
    if (Topic.IsEmpty())
    {
        OutError = FString::Printf(TEXT("Sensor '%s': no valid ROS topic for logical key '%s'."), *RawSensorId, *LogicalKey);
        return 0;
    }

    const int32 PublisherId = Bridge->Advertise(Topic, MessageType, Qos);
    if (PublisherId <= 0)
    {
        OutError = FString::Printf(TEXT("Sensor '%s': the uBot ROS bridge rejected topic '%s' (%s)."), *RawSensorId, *Topic, *MessageType);
        return 0;
    }

    FAdvertisedTopic& Advertised = AdvertisedTopics.AddDefaulted_GetRef();
    Advertised.Id = PublisherId;
    Advertised.Topic = Topic;
    Advertised.Qos = Qos;
    return PublisherId;
}

FString UUBotROSSensorPublisher::ResolveTopic(const FString& LogicalKey, const FString& DefaultRelativePath) const
{
    if (PublisherContext.ResolveTopic)
    {
        return PublisherContext.ResolveTopic(LogicalKey, DefaultRelativePath);
    }
    const FString Topic = UBot::ROSPublishing::JoinTopic(PublisherContext.Namespace, DefaultRelativePath);
    return UBot::ROSPublishing::IsValidAbsoluteTopic(Topic) ? Topic : FString();
}

FString UUBotROSSensorPublisher::ResolveFrameId(const FString& RawFrameId) const
{
    if (PublisherContext.ResolveFrameId)
    {
        return PublisherContext.ResolveFrameId(RawFrameId);
    }
    return UBot::ROSPublishing::SanitizeRosPath(RawFrameId);
}

bool UUBotROSSensorPublisher::ShouldPublishSample(const FUBotFrameHeader& Header) const
{
    return bPublisherInitialized && (Header.bValid || PublisherContext.bPublishInvalidSamples);
}

bool UUBotROSSensorPublisher::WantsMessage(int32 PublisherId) const
{
    UUBotROSBridgeSubsystem* Bridge = BridgePtr.Get();
    if (!bPublisherInitialized || !Bridge || PublisherId <= 0)
    {
        return false;
    }
    // -1 means the bridge has not reported a count yet: publish, the data is queued or dropped per QoS.
    if (PublisherContext.bSkipWhenNoSubscribers && Bridge->GetSubscriptionCount(PublisherId) == 0)
    {
        return false;
    }
    const FAdvertisedTopic* Advertised = FindAdvertisedTopic(PublisherId);
    if (Advertised && Advertised->Qos.Reliability == EUBotROSReliability::BestEffort && !Bridge->IsBridgeConnected())
    {
        return false;
    }
    return true;
}

bool UUBotROSSensorPublisher::PublishCdr(int32 PublisherId, TArray<uint8>&& Cdr)
{
    UUBotROSBridgeSubsystem* Bridge = BridgePtr.Get();
    return Bridge && PublisherId > 0 && Bridge->Publish(PublisherId, MoveTemp(Cdr));
}

const UUBotROSSensorPublisher::FAdvertisedTopic* UUBotROSSensorPublisher::FindAdvertisedTopic(int32 PublisherId) const
{
    return AdvertisedTopics.FindByPredicate([PublisherId](const FAdvertisedTopic& Advertised)
    {
        return Advertised.Id == PublisherId;
    });
}

namespace UBot::ROSPublishing
{
    FName GetExtensionPointName()
    {
        return FName(ExtensionPointName);
    }

    TArray<TSharedRef<IUBotROSSensorPublisherFactory>> FindFactories(const UObject* Sensor)
    {
        TArray<TSharedRef<IUBotROSSensorPublisherFactory>> Supported;
        if (!Sensor)
        {
            return Supported;
        }
        for (const TSharedRef<IUBotROSSensorPublisherFactory>& Factory : FUBotExtensionRegistry::Get().GetExtensionsAs<IUBotROSSensorPublisherFactory>(GetExtensionPointName()))
        {
            if (Factory->SupportsSensor(Sensor))
            {
                Supported.Add(Factory);
            }
        }
        Algo::StableSort(Supported, [](const TSharedRef<IUBotROSSensorPublisherFactory>& A, const TSharedRef<IUBotROSSensorPublisherFactory>& B)
        {
            return A->GetPriority() > B->GetPriority();
        });
        return Supported;
    }

    TSharedPtr<IUBotROSSensorPublisherFactory> FindFactory(const UObject* Sensor)
    {
        TArray<TSharedRef<IUBotROSSensorPublisherFactory>> Supported = FindFactories(Sensor);
        if (Supported.IsEmpty())
        {
            return nullptr;
        }
        return Supported[0];
    }

    FString SanitizeRosPath(const FString& Path)
    {
        FString Normalized = Path.TrimStartAndEnd();
        Normalized.ReplaceInline(TEXT("\\"), TEXT("/"), ESearchCase::CaseSensitive);

        TArray<FString> Segments;
        Normalized.ParseIntoArray(Segments, TEXT("/"), true);

        FString Result;
        for (const FString& Segment : Segments)
        {
            const FString Trimmed = Segment.TrimStartAndEnd();
            if (Trimmed.IsEmpty())
            {
                continue;
            }
            if (!Result.IsEmpty())
            {
                Result.AppendChar(TEXT('/'));
            }
            Result += UBot::ROSConventions::SanitizeRosName(Trimmed);
        }
        return Result;
    }

    FString JoinTopic(const FString& Namespace, const FString& RelativePath)
    {
        const FString SanitizedNamespace = SanitizeRosPath(Namespace);
        const FString SanitizedPath = SanitizeRosPath(RelativePath);
        FString Topic = TEXT("/");
        Topic += SanitizedNamespace;
        if (!SanitizedNamespace.IsEmpty() && !SanitizedPath.IsEmpty())
        {
            Topic.AppendChar(TEXT('/'));
        }
        Topic += SanitizedPath;
        return Topic;
    }

    bool IsValidAbsoluteTopic(const FString& Topic, FString* OutReason)
    {
        auto Reject = [OutReason](const TCHAR* Reason)
        {
            if (OutReason)
            {
                *OutReason = Reason;
            }
            return false;
        };

        if (!Topic.StartsWith(TEXT("/"), ESearchCase::CaseSensitive))
        {
            return Reject(TEXT("it is not an absolute topic (must start with '/')"));
        }
        if (Topic.Len() < 2)
        {
            return Reject(TEXT("it has no name after '/'"));
        }
        if (Topic.EndsWith(TEXT("/"), ESearchCase::CaseSensitive))
        {
            return Reject(TEXT("it ends with '/'"));
        }

        bool bSegmentStart = false;
        for (const TCHAR Character : Topic)
        {
            if (Character == TEXT('/'))
            {
                if (bSegmentStart)
                {
                    return Reject(TEXT("it contains an empty segment ('//')"));
                }
                bSegmentStart = true;
                continue;
            }
            const bool bAsciiAlnum = (Character >= TEXT('a') && Character <= TEXT('z'))
                || (Character >= TEXT('A') && Character <= TEXT('Z'))
                || (Character >= TEXT('0') && Character <= TEXT('9'));
            if (!bAsciiAlnum && Character != TEXT('_'))
            {
                return Reject(TEXT("it contains characters other than [A-Za-z0-9_/]"));
            }
            if (bSegmentStart && Character >= TEXT('0') && Character <= TEXT('9'))
            {
                return Reject(TEXT("a name segment starts with a digit"));
            }
            bSegmentStart = false;
        }

        // The checks above explain a rejection; the bridge protocol has the final word (e.g. length limit).
        if (!UBot::ROSProtocol::IsValidTopicName(Topic))
        {
            return Reject(TEXT("the uBot ROS bridge does not accept it (too long)"));
        }
        return true;
    }
}
