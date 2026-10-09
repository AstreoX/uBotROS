#pragma once

#include "CoreMinimal.h"
#include "Templates/Function.h"
#include "UObject/Object.h"
#include "UObject/WeakObjectPtrTemplates.h"
#include "UBotExtensionRegistry.h"
#include "UBotROSMessageTypes.h"
#include "UBotROSTypes.h"
#include "UBotROSSensorPublisher.generated.h"

class IUBotROSSensorPublisherFactory;
class UUBotROSBridgeSubsystem;
struct FUBotFrameHeader;

/**
 * Everything a sensor publisher needs from the component that owns it. The resolve functions are
 * provided by UUBotROSSensorBridgeComponent (topic overrides, conflict checks, frame prefixes); a
 * publisher that receives none falls back to "/<Namespace>/<DefaultRelativePath>" and sanitized frames.
 */
struct UBOTROS_API FUBotROSPublisherContext
{
    UUBotROSBridgeSubsystem* Bridge = nullptr;

    /** Sanitized robot namespace without leading or trailing slash, e.g. "robot_1" or "fleet/robot_1"; empty = root namespace. */
    FString Namespace;

    /** QoS of high-rate sensor data (IMU, scans, clouds, images). Default best_effort / volatile / 5. */
    FUBotROSQos SensorQos = FUBotROSQos::SensorData();

    /** QoS of state messages (pose, odometry). Default reliable / volatile / 10. */
    FUBotROSQos StateQos = FUBotROSQos::State();

    bool bPublishInvalidSamples = false;

    /** Skip encoding (and image readback) while the bridge reports 0 subscribers for a topic. */
    bool bSkipWhenNoSubscribers = true;

    /** Number of sensors handled by the same factory on the actor; > 1 switches defaults to the <SensorId>/... layout. */
    int32 SensorsOfSameType = 1;

    /**
     * LogicalKey is the TopicOverrides key, DefaultRelativePath the path below "/<Namespace>/". Returns the
     * absolute topic, or an empty string when the topic is invalid or already used (the caller reports why).
     */
    TFunction<FString(const FString& LogicalKey, const FString& DefaultRelativePath)> ResolveTopic;

    /** Raw frame id (e.g. a sensor FrameId) -> frame id as published in headers and TF. */
    TFunction<FString(const FString& RawFrameId)> ResolveFrameId;
};

/**
 * Publishes one uBot sensor to ROS. Created by an IUBotROSSensorPublisherFactory, owned by the
 * UUBotROSSensorBridgeComponent that discovered the sensor (UPROPERTY array, outer = component).
 *
 * Subclasses override Initialize (call Super::Initialize first, then advertise with AdvertiseTopic
 * and bind to the sensor's delegates with UFUNCTION handlers) and Shutdown (unbind, then call
 * Super::Shutdown, which unadvertises every topic advertised through AdvertiseTopic).
 */
UCLASS(Abstract)
class UBOTROS_API UUBotROSSensorPublisher : public UObject
{
    GENERATED_BODY()

public:
    /**
     * Base implementation: validates the arguments and caches the sensor, the bridge, the context,
     * the raw sensor id and the resolved sensor frame id. Returns false with OutError on failure.
     * Calling Initialize again shuts the previous setup down first.
     */
    virtual bool Initialize(UObject* Sensor, const FUBotROSPublisherContext& Context, FString& OutError);

    /** Unbinds from the sensor and unadvertises every topic. Idempotent. */
    virtual void Shutdown();

    /** Absolute topics this publisher advertised, in advertisement order. */
    virtual TArray<FString> GetTopics() const;

    /** True when the published frame is "<sensor frame>_optical" (cameras); the bridge component then adds the optical static TF. */
    virtual bool UsesOpticalFrame() const { return false; }

    /**
     * Whether the sensor's frame is a physical link of the robot, so that the bridge component publishes a
     * static transform from the base frame to it. False for sensors that only measure (the odometry sensor's
     * FrameId "odom" is the odometry frame, a parent of the base frame, not a child of it).
     */
    virtual bool PublishesPhysicalFrame() const { return true; }

    /** The sensor passed to Initialize, or nullptr after Shutdown or when it was destroyed. */
    UObject* GetSensor() const { return SensorPtr.Get(); }

    /** IUBotSensor::GetSensorId of the sensor, trimmed (raw, not sanitized). */
    const FString& GetSensorId() const { return RawSensorId; }

    /** Sanitized sensor id: a single ROS name token used for topic paths and override keys. */
    FString GetSensorToken() const;

    /** Context.ResolveFrameId(IUBotSensor::GetSensorFrameId): the sensor frame used in headers and TF. */
    const FString& GetSensorFrameId() const { return SensorFrameId; }

    /** GetSensorFrameId() + UBot::Ros::OpticalFrameSuffix(). */
    FString GetOpticalFrameId() const;

    bool IsPublisherInitialized() const { return bPublisherInitialized; }

protected:
    /**
     * Resolves LogicalKey / DefaultRelativePath through the context, advertises the topic and records
     * it for GetTopics and Shutdown. Returns the publisher id (> 0) or 0 with OutError.
     */
    int32 AdvertiseTopic(const FString& LogicalKey, const FString& DefaultRelativePath, const FString& MessageType, const FUBotROSQos& Qos, FString& OutError);

    /** Context.ResolveTopic, or "/<Namespace>/<sanitized DefaultRelativePath>" without one. */
    FString ResolveTopic(const FString& LogicalKey, const FString& DefaultRelativePath) const;

    /** Context.ResolveFrameId, or the sanitized frame without one. */
    FString ResolveFrameId(const FString& RawFrameId) const;

    /** Initialized, and the sample is valid or invalid samples are published. */
    bool ShouldPublishSample(const FUBotFrameHeader& Header) const;

    /**
     * Whether encoding a message for PublisherId is worthwhile: false without a bridge, when
     * bSkipWhenNoSubscribers and the bridge reports 0 subscribers, or when the topic is best_effort and
     * the bridge is not connected (the bridge drops such data while disconnected anyway).
     */
    bool WantsMessage(int32 PublisherId) const;

    /** Bridge->Publish; false when there is no bridge or the id is unknown. */
    bool PublishCdr(int32 PublisherId, TArray<uint8>&& Cdr);

    template <typename TMessage>
    bool PublishMessage(int32 PublisherId, const TMessage& Message)
    {
        return PublishCdr(PublisherId, UBot::Ros::Encode(Message));
    }

    bool IsSoleSensorOfType() const { return PublisherContext.SensorsOfSameType <= 1; }

    /** The context passed to Initialize. Its Bridge member is cleared (it is not GC-tracked); use GetBridge(). */
    const FUBotROSPublisherContext& GetContext() const { return PublisherContext; }

    UUBotROSBridgeSubsystem* GetBridge() const { return BridgePtr.Get(); }

private:
    struct FAdvertisedTopic
    {
        int32 Id = 0;
        FString Topic;
        FUBotROSQos Qos;
    };

    const FAdvertisedTopic* FindAdvertisedTopic(int32 PublisherId) const;

    TWeakObjectPtr<UObject> SensorPtr;
    TWeakObjectPtr<UUBotROSBridgeSubsystem> BridgePtr;
    FUBotROSPublisherContext PublisherContext;
    TArray<FAdvertisedTopic> AdvertisedTopics;
    FString RawSensorId;
    FString SensorFrameId;
    bool bPublisherInitialized = false;
};

/**
 * Creates publishers for the sensors it supports. Register implementations at the extension point
 * UBot::ROSPublishing::ExtensionPointName ("UBotROS.SensorPublisher") with
 * FUBotExtensionRegistry::RegisterExtension and remove them in ShutdownModule. For every discovered
 * sensor the bridge component uses the supporting factory with the highest priority; equal
 * priorities keep registration order, so the built-in factories (priority 0) win ties.
 */
class UBOTROS_API IUBotROSSensorPublisherFactory : public IUBotExtension
{
public:
    virtual int32 GetPriority() const { return 0; }
    virtual bool SupportsSensor(const UObject* Sensor) const = 0;
    virtual UUBotROSSensorPublisher* CreatePublisher(UObject* Outer, UObject* Sensor) const = 0;
};

namespace UBot::ROSPublishing
{
    /** Name of the sensor publisher extension point declared by UBotROS. */
    inline constexpr const TCHAR* ExtensionPointName = TEXT("UBotROS.SensorPublisher");

    /** Package that owns the extension point and the built-in factories. */
    inline constexpr const TCHAR* OwnerPackageName = TEXT("UBotROS");

    /** FName(ExtensionPointName). */
    UBOTROS_API FName GetExtensionPointName();

    /** Factories registered at the point that support Sensor, highest priority first (ties keep registration order). */
    UBOTROS_API TArray<TSharedRef<IUBotROSSensorPublisherFactory>> FindFactories(const UObject* Sensor);

    /** First entry of FindFactories, or nullptr when no factory supports Sensor. */
    UBOTROS_API TSharedPtr<IUBotROSSensorPublisherFactory> FindFactory(const UObject* Sensor);

    /**
     * ROS name path: whitespace trimmed, '\' treated as '/', empty segments dropped and every segment
     * passed through UBot::Ros::SanitizeRosName. "  /Fleet//robot-1/ " -> "Fleet/robot_1". Empty input -> "".
     */
    UBOTROS_API FString SanitizeRosPath(const FString& Path);

    /** "/" + Namespace + "/" + RelativePath, with sanitized parts and no duplicate slashes. */
    UBOTROS_API FString JoinTopic(const FString& Namespace, const FString& RelativePath);

    /**
     * Fully qualified ROS 2 topic: starts with '/', only [A-Za-z0-9_/], no empty segment, no trailing
     * '/', no segment starting with a digit, and accepted by the bridge protocol (at most 255 characters).
     * OutReason (optional) explains a rejection.
     */
    UBOTROS_API bool IsValidAbsoluteTopic(const FString& Topic, FString* OutReason = nullptr);
}
