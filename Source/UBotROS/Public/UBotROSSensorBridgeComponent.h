#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "Engine/EngineTypes.h"
#include "UObject/WeakObjectPtrTemplates.h"
#include "UBotROSTypes.h"
#include "UBotSensorTypes.h"
#include "UBotROSSensorBridgeComponent.generated.h"

class USceneComponent;
class UUBotOdometrySensorComponent;
class UUBotROSBridgeSubsystem;
class UUBotROSSensorPublisher;

DECLARE_DYNAMIC_MULTICAST_DELEGATE(FUBotROSSensorBridgeReadyEvent);
DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FUBotROSSensorBridgeErrorEvent, const FString&, ErrorMessage);
DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FUBotROSPublishersRefreshedEvent, int32, PublisherCount);

/**
 * Publishes every uBot sensor of its owner actor to ROS 2 through the uBot ROS bridge subsystem
 * (successor of the legacy AgentROSSensorSuitePublisher; same Blueprint API, events, status values
 * and validation rules).
 *
 * Discovery: UBot::Sensors::FindSensors(Owner) (the owner itself and its components; child actors are
 * not searched). For each sensor the supporting IUBotROSSensorPublisherFactory with the highest priority
 * creates a publisher; sensors no factory supports are skipped.
 *
 * Identity: namespace = RobotNamespaceOverride ("/" = root namespace), else UBot::Identity::ResolveNamespace(Owner); base frame
 * = BaseFrameOverride, else UBot::Identity::ResolveBaseFrameId(Owner); base component = BaseLinkComponent,
 * else UBot::Identity::ResolveBaseComponent(Owner). With bPrefixFramesWithNamespace every frame the
 * bridge publishes (except the world frame of world-referenced poses) becomes "<namespace>/<frame>".
 *
 * Topics: "/<namespace>/<default path>" (pose, odom, imu/data, scan, lidar/points,
 * camera/color/image_raw + camera_info, camera/depth/image_raw + camera_info; "<SensorId>/..." when several
 * sensors share a type) unless TopicOverrides has an enabled entry for the logical key (pose, odom, imu,
 * scan, lidar, rgb_camera, rgb_camera.camera_info, depth_camera, depth_camera.camera_info; for several
 * sensors of a type the sanitized SensorId and "<SensorId>.camera_info"). Overrides without a leading '/'
 * get one. Invalid topics, topics assigned twice, empty sensor ids or frame ids, duplicate sensor ids
 * within a type and duplicate sensor frame ids put the component into the Error state.
 *
 * TF: static base -> sensor (from IUBotSensor::GetSensorFrameComponent relative to the base component)
 * and sensor -> "<frame>_optical" for optical publishers through the subsystem's /tf_static aggregation;
 * dynamic odom -> base from the first odometry sensor whose tracked component is the base component and
 * whose frames match OdometryFrame / the base frame. The odometry sensor gets no static transform (its
 * FrameId is the odometry frame, which is the parent of the base frame), and neither does any other
 * sensor whose frame equals the odometry frame.
 *
 * The component becomes Ready once its publishers are set up; it does not wait for the bridge
 * connection (data is dropped or queued per QoS meanwhile) and logs one Warning per disconnected period.
 */
UCLASS(Blueprintable, ClassGroup = (uBot), meta = (BlueprintSpawnableComponent, DisplayName = "uBot ROS Sensor Bridge"))
class UBOTROS_API UUBotROSSensorBridgeComponent : public UActorComponent
{
    GENERATED_BODY()

public:
    UUBotROSSensorBridgeComponent();

    /**
     * Blank -> namespace of the owner's uBot identity (UBot::Identity::ResolveNamespace); "/" alone -> root
     * namespace (topics like "/imu/data"). '/' separates nested namespaces; every segment is sanitized.
     */
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "uBot ROS|Identity")
    FString RobotNamespaceOverride;

    /** Empty -> base frame of the owner's uBot identity (default "base_link"). */
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "uBot ROS|Identity")
    FString BaseFrameOverride;

    /** Empty or unresolved -> base component of the owner's uBot identity (default: root component). */
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "uBot ROS|Identity", meta = (UseComponentPicker, AllowedClasses = "/Script/Engine.SceneComponent"))
    FComponentReference BaseLinkComponent;

    /** Publish frames as "<namespace>/<frame>" (multi-robot TF trees). */
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "uBot ROS|Identity")
    bool bPrefixFramesWithNamespace = false;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "uBot ROS|Lifecycle")
    bool bAutoInitialize = true;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "uBot ROS|Publishing")
    bool bPublishInvalidSamples = false;

    /** Skip encoding (and camera readback) while the bridge reports no subscriber for a topic. */
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "uBot ROS|Publishing")
    bool bSkipWhenNoSubscribers = true;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "uBot ROS|TF")
    bool bPublishDynamicTF = true;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "uBot ROS|TF")
    bool bPublishStaticSensorTF = true;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "uBot ROS|TF")
    FString OdometryFrame = TEXT("odom");

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "uBot ROS|Topics", meta = (ToolTip = "Optional overrides keyed by logical topic name: pose, odom, imu, scan, lidar, rgb_camera, rgb_camera.camera_info, depth_camera, depth_camera.camera_info; for several sensors of one type the sanitized SensorId and <SensorId>.camera_info."))
    TMap<FString, FUBotROSTopicOverride> TopicOverrides;

    /** Overrides the sensor data QoS (default best_effort / volatile / 5). */
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "uBot ROS|QoS")
    FUBotROSQosOverride SensorDataQosOverride;

    /** Overrides the state QoS of pose and odometry (default reliable / volatile / 10). */
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "uBot ROS|QoS")
    FUBotROSQosOverride StateQosOverride;

    UPROPERTY(BlueprintAssignable, Category = "uBot ROS")
    FUBotROSSensorBridgeReadyEvent OnROSBridgeReady;

    UPROPERTY(BlueprintAssignable, Category = "uBot ROS")
    FUBotROSSensorBridgeErrorEvent OnROSBridgeError;

    UPROPERTY(BlueprintAssignable, Category = "uBot ROS")
    FUBotROSPublishersRefreshedEvent OnROSPublishersRefreshed;

    UFUNCTION(BlueprintPure, Category = "uBot ROS")
    bool IsROSBridgeReady() const { return Status == EUBotROSSensorBridgeStatus::Ready; }

    UFUNCTION(BlueprintPure, Category = "uBot ROS")
    EUBotROSSensorBridgeStatus GetROSBridgeStatus() const { return Status; }

    UFUNCTION(BlueprintPure, Category = "uBot ROS")
    FString GetROSBridgeError() const { return LastError; }

    /** Sensors that have a publisher. */
    UFUNCTION(BlueprintPure, Category = "uBot ROS")
    int32 GetDiscoveredSensorCount() const { return DiscoveredSensors.Num(); }

    /** Every advertised topic, sorted. */
    UFUNCTION(BlueprintPure, Category = "uBot ROS")
    TArray<FString> GetActivePublisherTopics() const;

    /** Namespace used for default topics (sanitized, no leading slash); empty for the root namespace. */
    UFUNCTION(BlueprintPure, Category = "uBot ROS")
    FString GetResolvedNamespace() const;

    /** Base frame as published (prefixed when bPrefixFramesWithNamespace). */
    UFUNCTION(BlueprintPure, Category = "uBot ROS")
    FString GetResolvedBaseFrameId() const;

    /** Re-discovers the owner's sensors and recreates every publisher; initializes the bridge when it is not running. */
    UFUNCTION(BlueprintCallable, Category = "uBot ROS")
    bool RefreshSensorPublishers();

    UFUNCTION(BlueprintCallable, Category = "uBot ROS")
    bool RestartROSBridge();

    UFUNCTION(BlueprintCallable, Category = "uBot ROS")
    void StopROSBridge();

    /** Raw frame id -> frame id as published (sanitized ROS path, prefixed when bPrefixFramesWithNamespace). Empty for an empty frame. */
    FString ResolveFrameId(const FString& RawFrameId) const;

    /** Current publishers, one per discovered sensor, in discovery order. */
    TArray<UUBotROSSensorPublisher*> GetPublishers() const;

    /** The publisher created for Sensor, or nullptr. */
    UUBotROSSensorPublisher* FindPublisherForSensor(const UObject* Sensor) const;

    /** The odometry sensor driving the dynamic odom -> base TF, or nullptr. */
    UUBotOdometrySensorComponent* GetDynamicTFOdometrySensor() const { return DynamicTFOdometrySensor.Get(); }

    //~ Begin UActorComponent Interface
    virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;
    //~ End UActorComponent Interface

protected:
    virtual void BeginPlay() override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

private:
    bool InitializeBridge();
    /** Discovers, validates and creates the publishers; returns false with OutError (the caller tears down and reports). */
    bool ConfigurePublishers(FString& OutError);
    void TearDownPublishers();
    void ConfigureDynamicTransform();
    void PublishStaticTransforms();
    void SetError(const FString& ErrorMessage);
    void SetReady();
    void StopConnectionMonitor();

    /** Resolves the namespace / base frame from the overrides and the owner identity into the Resolved* members. */
    bool ResolveIdentity(FString& OutError);
    /** Namespace from the current properties and identity (sanitized; empty when nothing usable is set). */
    FString ComputeNamespace() const;
    FString GetRawBaseFrameId() const;
    FString GetRawOdometryFrameId() const;
    USceneComponent* ResolveBaseComponent() const;
    FUBotROSQos ResolveQos(const FUBotROSQosOverride& Override, const FUBotROSQos& Default) const;

    /** Context.ResolveTopic implementation: overrides, validation and conflict detection for publisher PublisherIndex. */
    FString ResolveTopicForPublisher(const FString& LogicalKey, const FString& DefaultRelativePath, int32 PublisherIndex);

    /**
     * Validates Topic and claims it for PublisherIndex. A topic already claimed by another publisher is a
     * conflict; one claimed by PublisherIndex itself is accepted only when bAcceptOwnClaim. Sets
     * PendingTopicError and returns false on failure.
     */
    bool ClaimTopic(const FString& Topic, int32 PublisherIndex, bool bAcceptOwnClaim);

    /** Validates and claims the topics a newly initialized publisher reports (also those it advertised without the resolver). */
    bool ClaimPublisherTopics(const UUBotROSSensorPublisher& Publisher, int32 PublisherIndex);

    UFUNCTION()
    void HandleOdometryTransform(const FUBotOdometrySensorData& Data);

    UPROPERTY(Transient)
    TArray<TObjectPtr<UUBotROSSensorPublisher>> Publishers;

    UPROPERTY(Transient)
    TArray<TObjectPtr<UObject>> DiscoveredSensors;

    UPROPERTY(Transient)
    EUBotROSSensorBridgeStatus Status = EUBotROSSensorBridgeStatus::Uninitialized;

    UPROPERTY(Transient)
    FString LastError;

    UPROPERTY(Transient)
    TArray<FString> ActivePublisherTopics;

    TWeakObjectPtr<UUBotROSBridgeSubsystem> Bridge;
    TWeakObjectPtr<UUBotOdometrySensorComponent> DynamicTFOdometrySensor;

    struct FClaimedTopic
    {
        FString Topic;
        int32 PublisherIndex = INDEX_NONE;
    };

    /** Topics claimed during the current configuration (case-sensitive, like ROS names). */
    TArray<FClaimedTopic> ClaimedTopics;
    FString PendingTopicError;

    FString ResolvedNamespace;
    FString ResolvedBaseFrameId;
    FString ResolvedOdometryFrameId;

    /** Connection monitor (component tick while Ready). */
    double DisconnectedSinceSeconds = -1.0;
    bool bWarnedDisconnected = false;
};
