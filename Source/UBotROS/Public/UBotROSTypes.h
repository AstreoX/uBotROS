#pragma once

#include "CoreMinimal.h"
#include "UBotROSTypes.generated.h"

/** ROS 2 QoS reliability policy. */
UENUM(BlueprintType)
enum class EUBotROSReliability : uint8
{
    BestEffort UMETA(DisplayName = "Best Effort"),
    Reliable
};

/** ROS 2 QoS durability policy. TransientLocal is used for latched topics such as /tf_static. */
UENUM(BlueprintType)
enum class EUBotROSDurability : uint8
{
    Volatile,
    TransientLocal UMETA(DisplayName = "Transient Local")
};

/** State of the TCP connection between a world and ubot_ros_bridge. */
UENUM(BlueprintType)
enum class EUBotROSBridgeStatus : uint8
{
    Disabled,
    Connecting,
    Connected,
    Error
};

/**
 * Lifecycle of a sensor bridge component. Values match the legacy EAgentROSBridgeStatus one to one so
 * that an enum CoreRedirect needs no value redirects.
 */
UENUM(BlueprintType)
enum class EUBotROSSensorBridgeStatus : uint8
{
    Uninitialized,
    Initializing,
    Ready,
    Error,
    Stopped
};

/** ROS 2 QoS profile as sent to the bridge (history is always KEEP_LAST with Depth). */
USTRUCT(BlueprintType)
struct UBOTROS_API FUBotROSQos
{
    GENERATED_BODY()

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "uBot|ROS")
    EUBotROSReliability Reliability = EUBotROSReliability::BestEffort;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "uBot|ROS")
    EUBotROSDurability Durability = EUBotROSDurability::Volatile;

    /** KEEP_LAST history depth. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "uBot|ROS", meta = (ClampMin = "1", ClampMax = "1000", UIMin = "1", UIMax = "1000"))
    int32 Depth = 5;

    FUBotROSQos() = default;

    FUBotROSQos(EUBotROSReliability InReliability, EUBotROSDurability InDurability, int32 InDepth)
        : Reliability(InReliability)
        , Durability(InDurability)
        , Depth(InDepth)
    {
    }

    /** Sensor streams: best_effort / volatile / 5 (the default-constructed value). */
    static FUBotROSQos SensorData()
    {
        return FUBotROSQos(EUBotROSReliability::BestEffort, EUBotROSDurability::Volatile, 5);
    }

    /** State topics such as pose and odometry: reliable / volatile / 10. */
    static FUBotROSQos State()
    {
        return FUBotROSQos(EUBotROSReliability::Reliable, EUBotROSDurability::Volatile, 10);
    }

    /** Latched topics such as /tf_static: reliable / transient_local / 1. */
    static FUBotROSQos Latched()
    {
        return FUBotROSQos(EUBotROSReliability::Reliable, EUBotROSDurability::TransientLocal, 1);
    }

    /** Dynamic transforms on /tf: reliable / volatile / 100. */
    static FUBotROSQos Tf()
    {
        return FUBotROSQos(EUBotROSReliability::Reliable, EUBotROSDurability::Volatile, 100);
    }

    bool operator==(const FUBotROSQos& Other) const
    {
        return Reliability == Other.Reliability && Durability == Other.Durability && Depth == Other.Depth;
    }

    bool operator!=(const FUBotROSQos& Other) const
    {
        return !(*this == Other);
    }
};

/** Optional replacement of a default topic. Same fields as the legacy FAgentROSTopicOverride. */
USTRUCT(BlueprintType)
struct UBOTROS_API FUBotROSTopicOverride
{
    GENERATED_BODY()

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "uBot|ROS")
    bool bOverride = false;

    /** Topic used instead of the default when bOverride is set; the owning component resolves it. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "uBot|ROS", meta = (EditCondition = "bOverride"))
    FString Topic;
};

/**
 * Optional replacement of a default QoS profile.
 *
 * Successor of the legacy FAgentROSQOSOverride {bOverride, Reliability, QueueSize} (struct redirect in
 * Config/DefaultUBotROS.ini). The legacy Reliability and QueueSize values load into the two load-only
 * members below (UHT reflects them as Reliability / QueueSize, so the old tags match by name) and are
 * moved into Qos by PostSerialize, so existing assets keep their overrides.
 */
USTRUCT(BlueprintType)
struct UBOTROS_API FUBotROSQosOverride
{
    GENERATED_BODY()

    /** Value of Reliability_DEPRECATED while no legacy data was loaded (not a valid EUBotROSReliability). */
    static constexpr uint8 LegacyReliabilityAbsent = 255;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "uBot|ROS")
    bool bOverride = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "uBot|ROS", meta = (EditCondition = "bOverride"))
    FUBotROSQos Qos;

    /** Load-only (legacy FAgentROSQOSOverride::Reliability); never saved. LegacyReliabilityAbsent = not loaded. */
    UPROPERTY()
    EUBotROSReliability Reliability_DEPRECATED = static_cast<EUBotROSReliability>(LegacyReliabilityAbsent);

    /** Load-only (legacy FAgentROSQOSOverride::QueueSize); never saved. 0 = not loaded (the legacy range was 1..1000). */
    UPROPERTY()
    int32 QueueSize_DEPRECATED = 0;

    /** Moves loaded legacy values into Qos and clears the load-only members. */
    void PostSerialize(const FArchive& Ar);
};

template <>
struct TStructOpsTypeTraits<FUBotROSQosOverride> : public TStructOpsTypeTraitsBase2<FUBotROSQosOverride>
{
    enum
    {
        WithPostSerialize = true
    };
};
