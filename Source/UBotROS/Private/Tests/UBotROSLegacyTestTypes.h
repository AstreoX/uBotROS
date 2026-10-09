#pragma once

#include "CoreMinimal.h"
#include "UBotROSTypes.h"
#include "UBotROSLegacyTestTypes.generated.h"

// Test-only reflected types for UBotROS.Redirects.* (UHT cannot see types declared in a .cpp, and it does not
// allow WITH_DEV_AUTOMATION_TESTS guards, so these are compiled in every configuration).

/**
 * Same property names and types as the legacy FAgentROSQOSOverride {bOverride, Reliability, QueueSize}
 * (with the reliability enum already under its new name). Serializing it with tagged properties produces
 * data in the legacy layout, which the test then loads into FUBotROSQosOverride.
 */
USTRUCT()
struct FUBotROSLegacyQosOverrideMirror
{
    GENERATED_BODY()

    UPROPERTY()
    bool bOverride = false;

    UPROPERTY()
    EUBotROSReliability Reliability = EUBotROSReliability::BestEffort;

    UPROPERTY()
    int32 QueueSize = 5;
};
