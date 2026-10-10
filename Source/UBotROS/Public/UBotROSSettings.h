#pragma once

#include "CoreMinimal.h"
#include "Engine/DeveloperSettings.h"
#include "UBotROSSettings.generated.h"

/**
 * Project settings for uBot ROS (Project Settings > Plugins > uBot ROS), stored in the project's
 * Config/DefaultUBotROS.ini, section [/Script/UBotROS.UBotROSSettings], where uBot Manager edits
 * BridgeHost and BridgePort too. Read when a world's bridge subsystem connects; changes apply on the
 * next Connect().
 */
UCLASS(Config = UBotROS, DefaultConfig, meta = (DisplayName = "uBot ROS"))
class UBOTROS_API UUBotROSSettings : public UDeveloperSettings
{
    GENERATED_BODY()

public:
    //~ Begin UDeveloperSettings interface
    virtual FName GetCategoryName() const override;
    //~ End UDeveloperSettings interface

    /** Host of the ubot_ros_bridge node (name or IP address). IPv4 results are preferred. */
    UPROPERTY(Config, EditAnywhere, Category = "Bridge")
    FString BridgeHost = TEXT("127.0.0.1");

    /** TCP port of the ubot_ros_bridge node. */
    UPROPERTY(Config, EditAnywhere, Category = "Bridge", meta = (ClampMin = "1", ClampMax = "65535", UIMin = "1", UIMax = "65535"))
    int32 BridgePort = 8268;

    /** Connect automatically when a game or PIE world begins play. */
    UPROPERTY(Config, EditAnywhere, Category = "Bridge")
    bool bAutoConnect = true;

    /** Delay between connection attempts while the bridge is unreachable or after the connection is lost. */
    UPROPERTY(Config, EditAnywhere, Category = "Bridge", meta = (ClampMin = "0.05", Units = "s"))
    double ReconnectIntervalSeconds = 2.0;

    /** Publish rosgraph_msgs/msg/Clock with the uBot sim time every world tick while connected. */
    UPROPERTY(Config, EditAnywhere, Category = "Clock")
    bool bPublishClock = true;

    /** Absolute topic of the clock publisher. */
    UPROPERTY(Config, EditAnywhere, Category = "Clock", meta = (EditCondition = "bPublishClock"))
    FString ClockTopic = TEXT("/clock");

    /** Largest frame payload accepted from the bridge or sent to it (protocol default 64 MiB). */
    UPROPERTY(Config, EditAnywhere, Category = "Limits", AdvancedDisplay, meta = (ClampMin = "1024"))
    int32 MaxPayloadBytes = 67108864;

    /**
     * Budget for frames waiting to be sent (and for received messages waiting for dispatch). When
     * exceeded, the oldest pending best_effort messages are dropped first, then new reliable
     * messages are rejected with a warning. A value below MaxPayloadBytes + 12 (one maximum frame) is
     * raised to that when connecting (with a warning): otherwise large messages could never fit.
     */
    UPROPERTY(Config, EditAnywhere, Category = "Limits", AdvancedDisplay, meta = (ClampMin = "65536"))
    int64 MaxQueuedBytes = 268435456;
};
