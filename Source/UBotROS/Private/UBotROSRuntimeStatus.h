#pragma once

#include "CoreMinimal.h"
#include "UBotROSTypes.h"

/**
 * The bridge connection as a uBot runtime status item and problem (see UBotRuntimeStatus.h).
 *
 * There is one bridge subsystem per game or PIE world, but one "ros.bridge" item and one problem. Every
 * subsystem therefore publishes under its own Owner key (its address) and the item shows the aggregate:
 * the state of the most connected bridge (connected, else connecting, else failed, else disconnected; the
 * one that changed last when they tie). A world that goes away only withdraws its own state. Game thread only.
 */
namespace UBot::ROSRuntimeStatus
{
    /** Id of the runtime item and of the problem. */
    inline const TCHAR* const BridgeId = TEXT("ros.bridge");

    /**
     * Publishes Owner's state and refreshes the "ros.bridge" item (connecting, connected or disconnected,
     * with Endpoint as detail). When Owner's state decides the aggregate, Error reports the problem
     * rosBridgeUnreachable; Connected and Disabled clear it.
     */
    void PublishBridgeStatus(const void* Owner, EUBotROSBridgeStatus Status, const FString& Endpoint);

    /**
     * Withdraws Owner's state; nothing happens when it never published. The item and the problem are
     * refreshed from the owners that remain, and removed when none is left.
     */
    void ClearBridgeStatus(const void* Owner);

    /** Number of owners that currently publish a state (for tests). */
    int32 GetPublishingOwnerCount();

    /** Simplified Chinese for the texts above. */
    void RegisterLocalizedText();
}
