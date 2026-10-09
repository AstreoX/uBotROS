#include "UBotROSTypes.h"

#include "Serialization/Archive.h"
#include "UBotROS.h"

void FUBotROSQosOverride::PostSerialize(const FArchive& Ar)
{
    if (!Ar.IsLoading())
    {
        return;
    }

    // Legacy FAgentROSQOSOverride data is the only source of these values; data in the current format
    // never contains them, so a struct without legacy data is left exactly as loaded.
    const bool bHasReliability = static_cast<uint8>(Reliability_DEPRECATED) != LegacyReliabilityAbsent;
    const bool bHasQueueSize = QueueSize_DEPRECATED != 0;
    if (!bHasReliability && !bHasQueueSize)
    {
        return;
    }

    if (bHasReliability)
    {
        Qos.Reliability = Reliability_DEPRECATED;
    }
    if (bHasQueueSize)
    {
        Qos.Depth = FMath::Clamp(QueueSize_DEPRECATED, 1, 1000);
    }
    if (bOverride)
    {
        UE_LOG(LogUBotROS, Warning, TEXT("Migrated a legacy QoS override (reliability %s, depth %d) while loading %s; save the asset to store it in the new format."),
            Qos.Reliability == EUBotROSReliability::Reliable ? TEXT("reliable") : TEXT("best_effort"), Qos.Depth, *Ar.GetArchiveName());
    }

    Reliability_DEPRECATED = static_cast<EUBotROSReliability>(LegacyReliabilityAbsent);
    QueueSize_DEPRECATED = 0;
}
