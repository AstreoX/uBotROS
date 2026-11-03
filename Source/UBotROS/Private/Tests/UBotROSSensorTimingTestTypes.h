#pragma once

#include "CoreMinimal.h"
#include "Engine/World.h"
#include "UBotPointCloudFrame.h"
#include "UBotSensorTypes.h"
#include "UObject/Object.h"
#include "UBotROSSensorTimingTestTypes.generated.h"

struct FUBotSensorTimingEvent
{
    FString Kind;
    double Stamp = 0.0;
    double WorldTime = 0.0;
    int64 Sequence = 0;
    bool bValid = false;
    bool bMeasured = false;
    int32 Points = 0;
};

// Reflected delegate receiver used only by the sensor timing experiment. UHT requires this type
// outside WITH_DEV_AUTOMATION_TESTS, like the other test delegate receivers in this module.
UCLASS(Transient, NotBlueprintable, HideDropdown)
class UUBotROSSensorTimingRecorder : public UObject
{
    GENERATED_BODY()

public:
    UWorld* World = nullptr;
    bool bMeasured = false;
    TArray<FUBotSensorTimingEvent> Events;

    void Record(const TCHAR* Kind, const FUBotFrameHeader& Header, int32 Points = 0)
    {
        FUBotSensorTimingEvent& Event = Events.AddDefaulted_GetRef();
        Event.Kind = Kind;
        Event.Stamp = Header.TimestampSeconds;
        Event.WorldTime = World->GetTimeSeconds();
        Event.Sequence = Header.Sequence;
        Event.bValid = Header.bValid;
        Event.bMeasured = bMeasured;
        Event.Points = Points;
    }

    UFUNCTION()
    void OnCloud(UUBotPointCloudFrame* Frame)
    {
        if (Frame) Record(TEXT("cloud"), Frame->Header, Frame->HitPointCount);
    }

    UFUNCTION()
    void OnOdometry(const FUBotOdometrySensorData& Data)
    {
        Record(TEXT("odom"), Data.Header);
    }

    UFUNCTION()
    void OnImu(const FUBotImuSensorData& Data)
    {
        Record(TEXT("imu"), Data.Header);
    }
};
