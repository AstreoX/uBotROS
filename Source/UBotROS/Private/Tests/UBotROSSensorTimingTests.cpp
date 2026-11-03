#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Components/BoxComponent.h"
#include "Components/SceneComponent.h"
#include "Dom/JsonObject.h"
#include "Engine/Engine.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "GameFramework/Actor.h"
#include "HAL/FileManager.h"
#include "HAL/PlatformMisc.h"
#include "HAL/PlatformProcess.h"
#include "HAL/PlatformTime.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Misc/ScopeExit.h"
#include "Serialization/JsonSerializer.h"
#include "UBot3DLidarSensorComponent.h"
#include "UBotImuSensorComponent.h"
#include "UBotOdometrySensorComponent.h"
#include "UBotROSBridgeSubsystem.h"
#include "UBotROSSensorBridgeComponent.h"
#include "UBotROSSensorTimingTestTypes.h"
#include "UBotROSSettings.h"

namespace UBotROSSensorTimingTests
{
    struct FCase
    {
        int32 Index;
        int32 Fps;
        int32 OdomHz;
        int32 CloudHz;
        bool bLidarFirst;
        int32 ForcedOrder = 0; // 0: natural UE scheduling, 1: odom before cloud, 2: reverse
        bool bShiftedStart = false;
        bool bJitter = false;
        bool bDenseCloud = false;
    };

    TArray<FCase> MakeCases()
    {
        TArray<FCase> Cases;
        const int32 Rates[][2] = {{50, 10}, {30, 10}, {10, 30}, {50, 13}};
        for (const int32 Fps : {20, 60, 190})
            for (const auto& Rate : Rates)
                for (const bool bLidarFirst : {false, true})
                    Cases.Add({Cases.Num(), Fps, Rate[0], Rate[1], bLidarFirst});
        Cases.Add({24, 190, 50, 10, false, 1});
        Cases.Add({25, 190, 50, 10, true, 2});
        Cases.Add({26, 190, 50, 10, false, 0, true});
        Cases.Add({27, 60, 50, 13, true, 0, false, true});
        Cases.Add({28, 190, 50, 10, false, 0, false, false, true});
        Cases.Add({29, 190, 50, 10, true, 0, false, false, true});
        return Cases;
    }

    struct FSettingsScope
    {
        UUBotROSSettings* Settings = GetMutableDefault<UUBotROSSettings>();
        bool bAutoConnect = Settings->bAutoConnect;
        bool bClock = Settings->bPublishClock;
        FString ClockTopic = Settings->ClockTopic;
        FSettingsScope()
        {
            Settings->bAutoConnect = false;
            Settings->bPublishClock = true;
            Settings->ClockTopic = TEXT("/ubot_timing/clock");
        }
        ~FSettingsScope()
        {
            Settings->bAutoConnect = bAutoConnect;
            Settings->bPublishClock = bClock;
            Settings->ClockTopic = ClockTopic;
        }
    };

    struct FWorldScope
    {
        UWorld* World = UWorld::CreateWorld(EWorldType::Game, false);
        uint64 SavedFrame = GFrameCounter;
        FWorldScope()
        {
            World->AddToRoot();
            GEngine->CreateNewWorldContext(EWorldType::Game).SetCurrentWorld(World);
            World->InitializeActorsForPlay(FURL());
        }
        ~FWorldScope()
        {
            for (TActorIterator<AActor> It(World); It; ++It)
                It->RouteEndPlay(EEndPlayReason::LevelTransition);
            GEngine->DestroyWorldContext(World);
            World->DestroyWorld(false);
            World->RemoveFromRoot();
            GFrameCounter = SavedFrame;
        }
        void Tick(float Dt)
        {
            World->Tick(LEVELTICK_All, Dt);
            ++GFrameCounter;
        }
    };

    template <typename T>
    T* AddSensor(AActor* Actor, USceneComponent* Root, const TCHAR* Name, float Hz)
    {
        T* Sensor = NewObject<T>(Actor, Name);
        Actor->AddInstanceComponent(Sensor);
        Sensor->SetupAttachment(Root);
        Sensor->UpdateRateHz = Hz;
        Sensor->bAutoStart = false;
        Sensor->bNoiseEnabled = false;
        return Sensor;
    }

    TSharedRef<FJsonObject> EventJson(const FUBotSensorTimingEvent& Event)
    {
        TSharedRef<FJsonObject> Result = MakeShared<FJsonObject>();
        Result->SetStringField(TEXT("kind"), Event.Kind);
        const FUBotRosTime Stamp = FUBotRosTime::FromSeconds(Event.Stamp);
        Result->SetNumberField(TEXT("stamp_ns"), static_cast<double>(Stamp.sec) * 1.e9 + Stamp.nanosec);
        Result->SetNumberField(TEXT("world_time"), Event.WorldTime);
        Result->SetNumberField(TEXT("stamp"), Event.Stamp);
        Result->SetNumberField(TEXT("sequence"), Event.Sequence);
        Result->SetBoolField(TEXT("valid"), Event.bValid);
        Result->SetBoolField(TEXT("measured"), Event.bMeasured);
        Result->SetNumberField(TEXT("points"), Event.Points);
        return Result;
    }
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotROSSensorTimingMatrixTest, "UBotROS.EndToEnd.SensorTiming",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotROSSensorTimingMatrixTest::RunTest(const FString& Parameters)
{
    using namespace UBotROSSensorTimingTests;
    if (FPlatformMisc::GetEnvironmentVariable(TEXT("UBOT_ROS_TIMING")) != TEXT("1"))
    {
        AddInfo(TEXT("SensorTiming skipped: set UBOT_ROS_TIMING=1 and start the ROS timing probes. See test/sensor_timing_probe.py."));
        return true;
    }
    const FString OutputPath = FPlatformMisc::GetEnvironmentVariable(TEXT("UBOT_ROS_TIMING_OUT"));
    if (!TestFalse(TEXT("UBOT_ROS_TIMING_OUT names a JSON output file"), OutputPath.IsEmpty())) return false;
    const FString Host = FPlatformMisc::GetEnvironmentVariable(TEXT("UBOT_ROS_TIMING_HOST"));
    const FString PortString = FPlatformMisc::GetEnvironmentVariable(TEXT("UBOT_ROS_TIMING_PORT"));
    const int32 Port = PortString.IsEmpty() ? 8278 : FCString::Atoi(*PortString);
    const FString CaseFilter = FPlatformMisc::GetEnvironmentVariable(TEXT("UBOT_ROS_TIMING_CASES"));
    TArray<FString> Selected;
    CaseFilter.ParseIntoArray(Selected, TEXT(","), true);
    FSettingsScope Settings;
    TArray<TSharedPtr<FJsonValue>> CasesJson;
    for (const FCase& Case : MakeCases())
    {
        if (!Selected.IsEmpty() && !Selected.Contains(FString::FromInt(Case.Index))) continue;
        FWorldScope Scope;
        UWorld* World = Scope.World;
        UUBotROSBridgeSubsystem* Subsystem = UUBotROSBridgeSubsystem::Get(World);
        if (!TestNotNull(TEXT("Bridge subsystem"), Subsystem)) return false;

        // A real collision target supplies nonempty scans; no synthetic frames or sensor stand-ins.
        AActor* Wall = World->SpawnActor<AActor>();
        UBoxComponent* Box = NewObject<UBoxComponent>(Wall);
        Wall->SetRootComponent(Box);
        Box->SetBoxExtent(FVector(10.0, 1500.0, 1500.0));
        Box->SetCollisionEnabled(ECollisionEnabled::QueryOnly);
        Box->SetCollisionResponseToAllChannels(ECR_Block);
        Box->RegisterComponent();
        Wall->SetActorLocation(FVector(1000.0, 0.0, 0.0));
        Wall->DispatchBeginPlay();
        if (Case.bDenseCloud)
        {
            // Close the horizontal field so all 16 x 360 rays return points, matching a full scan.
            for (int32 Side = 0; Side < 3; ++Side)
            {
                AActor* OtherWall = World->SpawnActor<AActor>();
                UBoxComponent* OtherBox = NewObject<UBoxComponent>(OtherWall);
                OtherWall->SetRootComponent(OtherBox);
                OtherBox->SetBoxExtent(Side == 0 ? FVector(10.0, 1500.0, 1500.0) : FVector(1500.0, 10.0, 1500.0));
                OtherBox->SetCollisionEnabled(ECollisionEnabled::QueryOnly);
                OtherBox->SetCollisionResponseToAllChannels(ECR_Block);
                OtherBox->RegisterComponent();
                OtherWall->SetActorLocation(Side == 0 ? FVector(-1000.0, 0.0, 0.0) : FVector(0.0, Side == 1 ? -1000.0 : 1000.0, 0.0));
                OtherWall->DispatchBeginPlay();
            }
        }

        AActor* Actor = World->SpawnActor<AActor>();
        USceneComponent* Root = NewObject<USceneComponent>(Actor, TEXT("Root"));
        Actor->SetRootComponent(Root);
        Root->RegisterComponent();
        const FString Namespace = FString::Printf(TEXT("ubot_timing/c%02d"), Case.Index);
        UUBotROSSensorTimingRecorder* Recorder = NewObject<UUBotROSSensorTimingRecorder>(Actor);
        Recorder->AddToRoot();
        ON_SCOPE_EXIT { Recorder->RemoveFromRoot(); };
        Recorder->World = World;
        UUBotOdometrySensorComponent* Odom = nullptr;
        UUBot3DLidarSensorComponent* Lidar = nullptr;
        const auto AddOdom = [&]()
        {
            Odom = AddSensor<UUBotOdometrySensorComponent>(Actor, Root, TEXT("Odometry"), Case.OdomHz);
            Odom->SensorId = TEXT("odometry");
            Odom->FrameId = TEXT("odom");
            Odom->OdometryFrameId = TEXT("odom");
            Odom->ChildFrameId = TEXT("base_link");
            Odom->OnOdometryDataUpdated.AddDynamic(Recorder, &UUBotROSSensorTimingRecorder::OnOdometry);
            Odom->RegisterComponent();
        };
        const auto AddLidar = [&]()
        {
            Lidar = AddSensor<UUBot3DLidarSensorComponent>(Actor, Root, TEXT("Lidar"), Case.CloudHz);
            Lidar->SensorId = TEXT("lidar");
            Lidar->FrameId = TEXT("lidar");
            Lidar->NumberOfChannels = Case.bDenseCloud ? 16 : 4;
            Lidar->HorizontalSamples = Case.bDenseCloud ? 360 : 24;
            Lidar->MaximumRangeMeters = 30.0f;
            Lidar->bEnableSmokeInterference = false;
            Lidar->SetRelativeLocation(FVector(0.0, 0.0, 12.0));
            Lidar->On3DLidarFrameReady.AddDynamic(Recorder, &UUBotROSSensorTimingRecorder::OnCloud);
            Lidar->RegisterComponent();
        };
        if (Case.bLidarFirst) { AddLidar(); AddOdom(); }
        else { AddOdom(); AddLidar(); }
        UUBotImuSensorComponent* Imu = AddSensor<UUBotImuSensorComponent>(Actor, Root, TEXT("Imu"), 100.0f);
        Imu->FrameId = TEXT("imu");
        Imu->OnImuDataUpdated.AddDynamic(Recorder, &UUBotROSSensorTimingRecorder::OnImu);
        Imu->RegisterComponent();
        UUBotROSSensorBridgeComponent* SensorBridge = NewObject<UUBotROSSensorBridgeComponent>(Actor);
        SensorBridge->RobotNamespaceOverride = Namespace;
        SensorBridge->bPrefixFramesWithNamespace = true;
        // Avoid subscriber-count feedback masking samples while measuring arrival order.
        SensorBridge->bSkipWhenNoSubscribers = false;
        SensorBridge->RegisterComponent();
        Actor->DispatchBeginPlay();
        if (!TestTrue(TEXT("Sensor bridge ready"), SensorBridge->IsROSBridgeReady())) return false;
        if (Case.ForcedOrder == 1) Lidar->AddTickPrerequisiteComponent(Odom);
        if (Case.ForcedOrder == 2) Odom->AddTickPrerequisiteComponent(Lidar);

        Subsystem->ConnectTo(Host.IsEmpty() ? TEXT("127.0.0.1") : Host, Port);
        const double ConnectDeadline = FPlatformTime::Seconds() + 8.0;
        while (!Subsystem->IsBridgeConnected() && FPlatformTime::Seconds() < ConnectDeadline)
        {
            Scope.Tick(0.01f);
            FPlatformProcess::Sleep(0.01f);
        }
        if (!TestTrue(TEXT("Timing bridge connected"), Subsystem->IsBridgeConnected())) return false;
        // Allow DDS discovery while the real sensors remain stopped.
        const double DiscoveryDeadline = FPlatformTime::Seconds() + 0.75;
        while (FPlatformTime::Seconds() < DiscoveryDeadline)
        {
            Scope.Tick(0.01f);
            FPlatformProcess::Sleep(0.01f);
        }
        const double StartSim = World->GetTimeSeconds();
        const double StartReal = FPlatformTime::Seconds();
        if (Case.bLidarFirst) { Lidar->StartSensor(); Odom->StartSensor(); }
        else { Odom->StartSensor(); if (!Case.bShiftedStart) Lidar->StartSensor(); }
        Imu->StartSensor();
        double ElapsedSim = 0.0;
        int32 Frames = 0;
        while (ElapsedSim < 4.0)
        {
            const float Dt = Case.bJitter && Frames % 7 == 6 ? 0.08f : 1.0f / Case.Fps;
            Recorder->bMeasured = ElapsedSim >= 1.0 && ElapsedSim < 3.5;
            Actor->SetActorLocation(FVector(20.0 * (World->GetTimeSeconds() + Dt - StartSim), 0.0, 0.0));
            Scope.Tick(Dt);
            ++Frames;
            ElapsedSim += Dt;
            if (Case.bShiftedStart && Frames == 1) Lidar->StartSensor();
            const double SleepFor = StartReal + ElapsedSim - FPlatformTime::Seconds();
            if (SleepFor > 0.0) FPlatformProcess::Sleep(static_cast<float>(SleepFor));
        }
        Odom->StopSensor();
        Lidar->StopSensor();
        Imu->StopSensor();
        // Keep the connection alive to flush queued frames without generating more sensor samples.
        for (int32 I = 0; I < 10; ++I) { Scope.Tick(0.01f); FPlatformProcess::Sleep(0.01f); }

        TSharedRef<FJsonObject> Result = MakeShared<FJsonObject>();
        Result->SetNumberField(TEXT("case"), Case.Index);
        Result->SetStringField(TEXT("namespace"), Namespace);
        Result->SetNumberField(TEXT("fps"), Case.Fps);
        Result->SetNumberField(TEXT("odom_hz"), Case.OdomHz);
        Result->SetNumberField(TEXT("cloud_hz"), Case.CloudHz);
        Result->SetBoolField(TEXT("lidar_first"), Case.bLidarFirst);
        Result->SetNumberField(TEXT("forced_order"), Case.ForcedOrder);
        Result->SetBoolField(TEXT("shifted_start"), Case.bShiftedStart);
        Result->SetBoolField(TEXT("jitter"), Case.bJitter);
        Result->SetBoolField(TEXT("dense_cloud"), Case.bDenseCloud);
        Result->SetNumberField(TEXT("motion_start"), StartSim);
        Result->SetNumberField(TEXT("wall_seconds"), FPlatformTime::Seconds() - StartReal);
        Result->SetNumberField(TEXT("odom_dropped_due"), Odom->GetDroppedSampleCount());
        Result->SetNumberField(TEXT("cloud_dropped_due"), Lidar->GetDroppedSampleCount());
        const FUBotROSBridgeStats Stats = Subsystem->GetStats();
        Result->SetNumberField(TEXT("transport_dropped"), Stats.DroppedMessages);
        Result->SetNumberField(TEXT("transport_rejected"), Stats.RejectedMessages);
        TArray<TSharedPtr<FJsonValue>> Events;
        int32 MeasuredClouds = 0;
        for (const FUBotSensorTimingEvent& Event : Recorder->Events)
        {
            Events.Add(MakeShared<FJsonValueObject>(EventJson(Event)));
            TestEqual(TEXT("Sample stamp equals its world's current time"), Event.Stamp, Event.WorldTime);
            if (Event.Kind == TEXT("cloud") && Event.bMeasured)
            {
                ++MeasuredClouds;
                TestTrue(TEXT("Real lidar sees collision geometry"), Event.Points > 0);
                if (Case.bDenseCloud) TestEqual(TEXT("Every dense scan returns 5760 real points"), Event.Points, 5760);
            }
        }
        TestTrue(TEXT("Measured at least 15 clouds"), MeasuredClouds >= 15);
        Result->SetArrayField(TEXT("events"), Events);
        CasesJson.Add(MakeShared<FJsonValueObject>(Result));
        AddInfo(FString::Printf(TEXT("Timing c%02d: %d fps, odom/cloud %d/%d Hz, lidar-first=%d, measured clouds=%d, transport drops=%lld"),
            Case.Index, Case.Fps, Case.OdomHz, Case.CloudHz, Case.bLidarFirst, MeasuredClouds, Stats.DroppedMessages));
        SensorBridge->StopROSBridge();
        Subsystem->Disconnect();
        TSharedRef<FJsonObject> Document = MakeShared<FJsonObject>();
        Document->SetArrayField(TEXT("cases"), CasesJson);
        FString Json;
        FJsonSerializer::Serialize(Document, TJsonWriterFactory<>::Create(&Json));
        IFileManager::Get().MakeDirectory(*FPaths::GetPath(OutputPath), true);
        if (!TestTrue(TEXT("Timing results saved"), FFileHelper::SaveStringToFile(Json, *OutputPath))) return false;
    }
    return true;
}

#endif
