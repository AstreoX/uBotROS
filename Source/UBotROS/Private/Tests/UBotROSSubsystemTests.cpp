#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Engine/Engine.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "HAL/PlatformProcess.h"
#include "HAL/PlatformTime.h"
#include "IPAddress.h"
#include "SocketSubsystem.h"
#include "Sockets.h"
#include "SocketTypes.h"
#include "UBotROSBridgeSubsystem.h"
#include "UBotROSMessageTypes.h"
#include "UBotROSProtocol.h"
#include "UBotROSRuntimeStatus.h"
#include "UBotROSSettings.h"
#include "UBotROSTypes.h"
#include "UBotRosConventions.h"
#include "UBotRuntimeStatus.h"

namespace UBotROSSubsystemTests
{
    using namespace UBot::ROSProtocol;

    constexpr double WaitTimeoutSeconds = 5.0;
    constexpr float TickSeconds = 1.0f / 60.0f;

    /**
     * Minimal in-process ubot_ros_bridge stand-in: listens on 127.0.0.1 with an ephemeral port, accepts
     * one client at a time (a new connection replaces the old one), records every frame of the current
     * connection and answers HELLO with HELLO_ACK and PING with PONG. Polled from the test (game) thread.
     */
    class FTestBridgeServer
    {
    public:
        UE_NONCOPYABLE(FTestBridgeServer);

        FTestBridgeServer()
        {
            ReceiveBuffer.SetNumUninitialized(256 * 1024);
        }

        ~FTestBridgeServer()
        {
            CloseClient();
            if (Listener != nullptr)
            {
                Listener->Close();
                Sockets->DestroySocket(Listener);
                Listener = nullptr;
            }
        }

        bool Start()
        {
            Sockets = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM);
            if (Sockets == nullptr)
            {
                return false;
            }
            Listener = Sockets->CreateSocket(NAME_Stream, TEXT("uBot ROS test bridge"), FNetworkProtocolTypes::IPv4);
            if (Listener == nullptr)
            {
                return false;
            }
            const TSharedRef<FInternetAddr> Address = Sockets->CreateInternetAddr(FNetworkProtocolTypes::IPv4);
            bool bValidIp = false;
            Address->SetIp(TEXT("127.0.0.1"), bValidIp);
            Address->SetPort(0);
            if (!bValidIp || !Listener->Bind(*Address) || !Listener->Listen(8))
            {
                return false;
            }
            Listener->SetNonBlocking(true);
            Port = Listener->GetPortNo();
            return Port > 0;
        }

        int32 GetPort() const
        {
            return Port;
        }

        /** Accepts a pending connection, reads everything available and answers HELLO / PING. */
        void Poll()
        {
            bool bPending = false;
            if (Listener != nullptr && Listener->HasPendingConnection(bPending) && bPending)
            {
                if (FSocket* Accepted = Listener->Accept(TEXT("uBot ROS test bridge client")))
                {
                    CloseClient();
                    Client = Accepted;
                    Client->SetNonBlocking(true);
                    Client->SetNoDelay(true);
                    Parser.Reset();
                    Frames.Reset();
                    ++AcceptCount;
                }
            }
            if (Client == nullptr)
            {
                return;
            }

            for (;;)
            {
                int32 BytesRead = 0;
                if (!Client->Recv(ReceiveBuffer.GetData(), ReceiveBuffer.Num(), BytesRead))
                {
                    ++ClientClosedCount;
                    CloseClient();
                    return;
                }
                if (BytesRead <= 0)
                {
                    return;
                }
                if (!Parser.Feed(ReceiveBuffer.GetData(), BytesRead))
                {
                    bProtocolError = true;
                    CloseClient();
                    return;
                }
                FWireFrame Frame;
                while (Parser.PopFrame(Frame))
                {
                    if (bAutoHelloAck && Frame.Op == static_cast<uint8>(EOp::Hello))
                    {
                        Send(BuildJsonFrame(EOp::HelloAck, MakeHelloAckJson(TEXT("test_bridge 0.0.0"), TEXT("humble"))));
                    }
                    else if (Frame.Op == static_cast<uint8>(EOp::Ping))
                    {
                        Send(BuildFrame(EOp::Pong, TConstArrayView<uint8>()));
                    }
                    Frames.Add(MoveTemp(Frame));
                }
            }
        }

        bool Send(TConstArrayView<uint8> Bytes)
        {
            if (Client == nullptr)
            {
                return false;
            }
            int32 Offset = 0;
            const double Deadline = FPlatformTime::Seconds() + 2.0;
            while (Offset < Bytes.Num())
            {
                int32 Sent = 0;
                if (Client->Send(Bytes.GetData() + Offset, Bytes.Num() - Offset, Sent) && Sent > 0)
                {
                    Offset += Sent;
                    continue;
                }
                if (FPlatformTime::Seconds() > Deadline)
                {
                    return false;
                }
                FPlatformProcess::Sleep(0.001f);
            }
            return true;
        }

        void CloseClient()
        {
            if (Client != nullptr)
            {
                Client->Close();
                Sockets->DestroySocket(Client);
                Client = nullptr;
            }
        }

        bool HasClient() const
        {
            return Client != nullptr;
        }

        int32 CountFrames(EOp Op) const
        {
            int32 Count = 0;
            for (const FWireFrame& Frame : Frames)
            {
                Count += Frame.Op == static_cast<uint8>(Op) ? 1 : 0;
            }
            return Count;
        }

        /** PUBLISH frames of one publisher id, in arrival order (bodies without the id prefix). */
        TArray<TArray<uint8>> PublishedBodies(uint32 PublisherId) const
        {
            TArray<TArray<uint8>> Bodies;
            for (const FWireFrame& Frame : Frames)
            {
                uint32 Id = 0;
                TConstArrayView<uint8> Body;
                if (Frame.Op == static_cast<uint8>(EOp::Publish) && SplitIdPayload(Frame.Payload, Id, Body) && Id == PublisherId)
                {
                    Bodies.Emplace(Body);
                }
            }
            return Bodies;
        }

        /** The last frame with Op on the current connection, or nullptr. */
        const FWireFrame* LastFrame(EOp Op) const
        {
            for (int32 Index = Frames.Num() - 1; Index >= 0; --Index)
            {
                if (Frames[Index].Op == static_cast<uint8>(Op))
                {
                    return &Frames[Index];
                }
            }
            return nullptr;
        }

        /** The ADVERTISE (or SUBSCRIBE) request for Topic, if received on the current connection. */
        bool FindTopicRequest(EOp Op, const FString& Topic, FTopicRequest& OutRequest) const
        {
            for (const FWireFrame& Frame : Frames)
            {
                FTopicRequest Request;
                if (Frame.Op == static_cast<uint8>(Op) && ParseTopicRequest(Frame.Payload, Request) && Request.Topic.Equals(Topic, ESearchCase::CaseSensitive))
                {
                    OutRequest = Request;
                    return true;
                }
            }
            return false;
        }

        TArray<FWireFrame> Frames;
        int32 AcceptCount = 0;
        int32 ClientClosedCount = 0;
        bool bProtocolError = false;
        bool bAutoHelloAck = true;

    private:
        ISocketSubsystem* Sockets = nullptr;
        FSocket* Listener = nullptr;
        FSocket* Client = nullptr;
        int32 Port = 0;
        FFrameParser Parser;
        TArray<uint8> ReceiveBuffer;
    };

    /**
     * Transient game world (with its bridge subsystem) and project settings that keep the subsystem
     * from auto-connecting to a real bridge; everything is restored when the scope ends.
     */
    class FScopedBridgeWorld
    {
    public:
        UE_NONCOPYABLE(FScopedBridgeWorld);

        explicit FScopedBridgeWorld(bool bPublishClock = false, int64 MaxQueuedBytes = 268435456, int32 MaxPayloadBytes = 67108864, double ReconnectIntervalSeconds = 0.1)
            : Settings(GetMutableDefault<UUBotROSSettings>())
        {
            Saved.bAutoConnect = Settings->bAutoConnect;
            Saved.bPublishClock = Settings->bPublishClock;
            Saved.ReconnectIntervalSeconds = Settings->ReconnectIntervalSeconds;
            Saved.MaxQueuedBytes = Settings->MaxQueuedBytes;
            Saved.MaxPayloadBytes = Settings->MaxPayloadBytes;
            Saved.ClockTopic = Settings->ClockTopic;

            Settings->bAutoConnect = false;
            Settings->bPublishClock = bPublishClock;
            Settings->ReconnectIntervalSeconds = ReconnectIntervalSeconds;
            Settings->MaxQueuedBytes = MaxQueuedBytes;
            Settings->MaxPayloadBytes = MaxPayloadBytes;
            Settings->ClockTopic = TEXT("/clock");

            World = UWorld::CreateWorld(EWorldType::Game, false);
            if (World != nullptr && GEngine != nullptr)
            {
                // A world context keeps UWorld::DestroyActor from warning about a world without one.
                GEngine->CreateNewWorldContext(EWorldType::Game).SetCurrentWorld(World);
            }
        }

        ~FScopedBridgeWorld()
        {
            if (World != nullptr)
            {
                if (GEngine != nullptr)
                {
                    GEngine->DestroyWorldContext(World);
                }
                // Deinitializes the subsystem, which joins its I/O thread.
                World->DestroyWorld(false);
                World = nullptr;
            }
            Settings->bAutoConnect = Saved.bAutoConnect;
            Settings->bPublishClock = Saved.bPublishClock;
            Settings->ReconnectIntervalSeconds = Saved.ReconnectIntervalSeconds;
            Settings->MaxQueuedBytes = Saved.MaxQueuedBytes;
            Settings->MaxPayloadBytes = Saved.MaxPayloadBytes;
            Settings->ClockTopic = Saved.ClockTopic;
        }

        UWorld* Get() const
        {
            return World;
        }

        UUBotROSBridgeSubsystem* Bridge() const
        {
            return UUBotROSBridgeSubsystem::Get(World);
        }

    private:
        struct FSavedSettings
        {
            bool bAutoConnect = true;
            bool bPublishClock = true;
            double ReconnectIntervalSeconds = 2.0;
            int64 MaxQueuedBytes = 0;
            int32 MaxPayloadBytes = 0;
            FString ClockTopic;
        };

        UUBotROSSettings* Settings = nullptr;
        FSavedSettings Saved;
        UWorld* World = nullptr;
    };

    /** Polls the server and (optionally) ticks the subsystem until Condition holds; false on timeout. */
    bool PumpUntil(FTestBridgeServer& Server, UUBotROSBridgeSubsystem* Bridge, TFunctionRef<bool()> Condition, bool bTickBridge = true, double TimeoutSeconds = WaitTimeoutSeconds)
    {
        const double Deadline = FPlatformTime::Seconds() + TimeoutSeconds;
        for (;;)
        {
            Server.Poll();
            if (bTickBridge && Bridge != nullptr)
            {
                Bridge->Tick(TickSeconds);
            }
            if (Condition())
            {
                return true;
            }
            if (FPlatformTime::Seconds() >= Deadline)
            {
                return false;
            }
            FPlatformProcess::Sleep(0.002f);
        }
    }

    /** Polls and ticks for a fixed time (to check that nothing else arrives). */
    void PumpFor(FTestBridgeServer& Server, UUBotROSBridgeSubsystem* Bridge, double Seconds)
    {
        PumpUntil(Server, Bridge, []() { return false; }, true, Seconds);
    }

    /** Connects Bridge to Server and waits for the handshake. */
    bool ConnectBridge(FAutomationTestBase& Test, FTestBridgeServer& Server, UUBotROSBridgeSubsystem* Bridge)
    {
        Bridge->ConnectTo(TEXT("127.0.0.1"), Server.GetPort());
        return Test.TestTrue(TEXT("Handshake completes"), PumpUntil(Server, Bridge, [Bridge]() { return Bridge->IsBridgeConnected(); }));
    }

    /** A complete serialized message: CDR_LE encapsulation header, then Size - 4 bytes starting with Marker. */
    TArray<uint8> FakeCdr(int32 Size, uint8 Marker)
    {
        TArray<uint8> Bytes;
        Bytes.SetNumZeroed(FMath::Max(Size, 5));
        Bytes[1] = 0x01;
        for (int32 Index = 4; Index < Bytes.Num(); ++Index)
        {
            Bytes[Index] = static_cast<uint8>(Marker + (Index - 4));
        }
        return Bytes;
    }

    FUBotRosTransformStamped MakeTestTransform(const TCHAR* Parent, const TCHAR* Child, double X, double Y, double Z)
    {
        FUBotRosTransformStamped Transform;
        Transform.header.stamp.sec = 3;
        Transform.header.stamp.nanosec = 500;
        Transform.header.frame_id = Parent;
        Transform.child_frame_id = Child;
        Transform.transform.translation.x = X;
        Transform.transform.translation.y = Y;
        Transform.transform.translation.z = Z;
        Transform.transform.rotation.w = 1.0;
        return Transform;
    }

    TArray<FString> ChildFrames(const FUBotRosTfMessage& Message)
    {
        TArray<FString> Children;
        for (const FUBotRosTransformStamped& Transform : Message.transforms)
        {
            Children.Add(Transform.child_frame_id);
        }
        return Children;
    }
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotROSSubsystemHandshakeTest, "UBotROS.Subsystem.HandshakeAndAdvertise", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotROSSubsystemHandshakeTest::RunTest(const FString& Parameters)
{
    using namespace UBot::ROSProtocol;
    using namespace UBotROSSubsystemTests;

    FTestBridgeServer Server;
    if (!TestTrue(TEXT("Test bridge listens on 127.0.0.1"), Server.Start()))
    {
        return false;
    }
    FScopedBridgeWorld Scope;
    UUBotROSBridgeSubsystem* Bridge = Scope.Bridge();
    if (!TestNotNull(TEXT("Bridge subsystem exists in a game world"), Bridge))
    {
        return false;
    }

    TestEqual(TEXT("Initially disabled"), Bridge->GetBridgeStatus(), EUBotROSBridgeStatus::Disabled);
    TestFalse(TEXT("Not connected"), Bridge->IsBridgeConnected());

    // Invalid names are refused with 0.
    AddExpectedErrorPlain(TEXT("cannot advertise"), EAutomationExpectedErrorFlags::Contains, 3);
    AddExpectedErrorPlain(TEXT("cannot subscribe"), EAutomationExpectedErrorFlags::Contains, 2);
    TestEqual(TEXT("Relative topic refused"), Bridge->Advertise(TEXT("imu"), TEXT("sensor_msgs/msg/Imu"), FUBotROSQos::SensorData()), 0);
    TestEqual(TEXT("Empty token refused"), Bridge->Advertise(TEXT("/a//imu"), TEXT("sensor_msgs/msg/Imu"), FUBotROSQos::SensorData()), 0);
    TestEqual(TEXT("Bad type refused"), Bridge->Advertise(TEXT("/a/imu"), TEXT("sensor_msgs/Imu"), FUBotROSQos::SensorData()), 0);
    TestEqual(TEXT("Subscription to a bad topic refused"), Bridge->Subscribe(TEXT("/a/9cmd"), TEXT("geometry_msgs/msg/Twist"), FUBotROSQos::SensorData(), [](TConstArrayView<uint8>) {}), 0);
    TestEqual(TEXT("Subscription without callback refused"), Bridge->Subscribe(TEXT("/a/cmd"), TEXT("geometry_msgs/msg/Twist"), FUBotROSQos::SensorData(), TFunction<void(TConstArrayView<uint8>)>()), 0);

    // Registered before connecting: sent right after HELLO.
    const int32 ImuId = Bridge->Advertise(TEXT("/ubot_test/imu"), TEXT("sensor_msgs/msg/Imu"), FUBotROSQos::SensorData());
    const int32 CmdId = Bridge->Subscribe(TEXT("/ubot_test/cmd_vel"), TEXT("geometry_msgs/msg/Twist"), FUBotROSQos(EUBotROSReliability::BestEffort, EUBotROSDurability::Volatile, 10), [](TConstArrayView<uint8>) {});
    TestEqual(TEXT("First publisher id"), ImuId, 1);
    TestEqual(TEXT("Subscription ids have their own counter"), CmdId, 1);
    TestTrue(TEXT("IsAdvertised"), Bridge->IsAdvertised(ImuId));
    TestTrue(TEXT("IsSubscribed"), Bridge->IsSubscribed(CmdId));

    int32 ConnectedEvents = 0;
    Bridge->OnBridgeConnectedNative.AddLambda([&ConnectedEvents]() { ++ConnectedEvents; });

    Bridge->ConnectTo(TEXT("127.0.0.1"), Server.GetPort());
    TestEqual(TEXT("Connecting right after ConnectTo"), Bridge->GetBridgeStatus(), EUBotROSBridgeStatus::Connecting);
    TestEqualSensitive(TEXT("Endpoint"), Bridge->GetBridgeEndpoint(), FString::Printf(TEXT("127.0.0.1:%d"), Server.GetPort()));
    if (!TestTrue(TEXT("Handshake completes"), PumpUntil(Server, Bridge, [Bridge]() { return Bridge->IsBridgeConnected(); })))
    {
        return false;
    }
    TestEqual(TEXT("Status Connected"), Bridge->GetBridgeStatus(), EUBotROSBridgeStatus::Connected);
    TestEqual(TEXT("OnBridgeConnected fired once"), ConnectedEvents, 1);
    PumpUntil(Server, Bridge, [&Server]() { return Server.Frames.Num() >= 3; });

    if (TestTrue(TEXT("HELLO, ADVERTISE and SUBSCRIBE received"), Server.Frames.Num() >= 3))
    {
        FHello Hello;
        TestEqual(TEXT("HELLO is the first frame"), Server.Frames[0].Op, static_cast<uint8>(EOp::Hello));
        if (TestTrue(TEXT("HELLO json"), ParseHello(Server.Frames[0].Payload, Hello)))
        {
            TestEqual(TEXT("HELLO protocol"), Hello.Protocol, 1);
            TestTrue(TEXT("HELLO client name"), Hello.Client.StartsWith(TEXT("uBot UE "), ESearchCase::CaseSensitive));
        }

        FTopicRequest Advertise;
        TestEqual(TEXT("ADVERTISE second"), Server.Frames[1].Op, static_cast<uint8>(EOp::Advertise));
        if (TestTrue(TEXT("ADVERTISE json"), ParseTopicRequest(Server.Frames[1].Payload, Advertise)))
        {
            TestEqual(TEXT("ADVERTISE id"), Advertise.Id, static_cast<uint32>(ImuId));
            TestEqualSensitive(TEXT("ADVERTISE topic"), Advertise.Topic, TEXT("/ubot_test/imu"));
            TestEqualSensitive(TEXT("ADVERTISE type"), Advertise.Type, TEXT("sensor_msgs/msg/Imu"));
            TestTrue(TEXT("ADVERTISE qos best_effort/volatile/5"), Advertise.Qos == FUBotROSQos::SensorData());
        }
        TestEqualSensitive(TEXT("ADVERTISE exact json"), FromUtf8(Server.Frames[1].Payload),
            TEXT("{\"id\":1,\"topic\":\"/ubot_test/imu\",\"type\":\"sensor_msgs/msg/Imu\",\"qos\":{\"reliability\":\"best_effort\",\"durability\":\"volatile\",\"depth\":5}}"));

        FTopicRequest Subscribe;
        TestEqual(TEXT("SUBSCRIBE third"), Server.Frames[2].Op, static_cast<uint8>(EOp::Subscribe));
        if (TestTrue(TEXT("SUBSCRIBE json"), ParseTopicRequest(Server.Frames[2].Payload, Subscribe)))
        {
            TestEqual(TEXT("SUBSCRIBE id"), Subscribe.Id, static_cast<uint32>(CmdId));
            TestEqualSensitive(TEXT("SUBSCRIBE topic"), Subscribe.Topic, TEXT("/ubot_test/cmd_vel"));
            TestEqualSensitive(TEXT("SUBSCRIBE type"), Subscribe.Type, TEXT("geometry_msgs/msg/Twist"));
            TestEqual(TEXT("SUBSCRIBE depth"), Subscribe.Qos.Depth, 10);
        }
    }

    // Registered while connected: sent immediately; ids keep counting up.
    const int32 OdomId = Bridge->Advertise(TEXT("/ubot_test/odom"), TEXT("nav_msgs/msg/Odometry"), FUBotROSQos::State());
    TestEqual(TEXT("Second publisher id"), OdomId, ImuId + 1);
    FTopicRequest OdomRequest;
    if (TestTrue(TEXT("ADVERTISE while connected"), PumpUntil(Server, Bridge, [&]() { return Server.FindTopicRequest(EOp::Advertise, TEXT("/ubot_test/odom"), OdomRequest); })))
    {
        TestEqual(TEXT("Odom id"), OdomRequest.Id, static_cast<uint32>(OdomId));
        TestTrue(TEXT("Odom qos reliable/volatile/10"), OdomRequest.Qos == FUBotROSQos::State());
    }

    Bridge->Unadvertise(OdomId);
    Bridge->Unsubscribe(CmdId);
    TestFalse(TEXT("Unadvertised"), Bridge->IsAdvertised(OdomId));
    TestFalse(TEXT("Unsubscribed"), Bridge->IsSubscribed(CmdId));
    if (TestTrue(TEXT("UNADVERTISE and UNSUBSCRIBE sent"), PumpUntil(Server, Bridge, [&Server]() { return Server.CountFrames(EOp::Unadvertise) == 1 && Server.CountFrames(EOp::Unsubscribe) == 1; })))
    {
        for (const FWireFrame& Frame : Server.Frames)
        {
            uint32 Id = 0;
            if (Frame.Op == static_cast<uint8>(EOp::Unadvertise))
            {
                TestTrue(TEXT("UNADVERTISE id"), ParseIdRequest(Frame.Payload, Id) && Id == static_cast<uint32>(OdomId));
                TestEqualSensitive(TEXT("UNADVERTISE exact json"), FromUtf8(Frame.Payload), FString::Printf(TEXT("{\"id\":%d}"), OdomId));
            }
            if (Frame.Op == static_cast<uint8>(EOp::Unsubscribe))
            {
                TestTrue(TEXT("UNSUBSCRIBE id"), ParseIdRequest(Frame.Payload, Id) && Id == static_cast<uint32>(CmdId));
            }
        }
    }
    const int32 NextSubscription = Bridge->Subscribe(TEXT("/ubot_test/cmd_vel"), TEXT("geometry_msgs/msg/Twist"), FUBotROSQos::SensorData(), [](TConstArrayView<uint8>) {});
    TestEqual(TEXT("Subscription ids are not reused"), NextSubscription, CmdId + 1);

    // The bridge pings: the client answers with PONG. The client pings on its own every 2 s.
    const int32 PongsBefore = Server.CountFrames(EOp::Pong);
    Server.Send(BuildFrame(EOp::Ping, TConstArrayView<uint8>()));
    TestTrue(TEXT("PING answered with PONG"), PumpUntil(Server, Bridge, [&Server, PongsBefore]() { return Server.CountFrames(EOp::Pong) > PongsBefore; }));
    TestTrue(TEXT("Client sends PING within 2 s"), PumpUntil(Server, Bridge, [&Server]() { return Server.CountFrames(EOp::Ping) > 0; }, true, 3.0));

    // ERROR frames are logged and do not break the connection.
    AddExpectedErrorPlain(TEXT("reported 'qos_conflict'"), EAutomationExpectedErrorFlags::Contains, 1);
    Server.Send(BuildJsonFrame(EOp::Error, MakeErrorJson(TOptional<uint32>(static_cast<uint32>(ImuId)), TEXT("qos_conflict"), TEXT("test"))));
    PumpFor(Server, Bridge, 0.1);
    TestTrue(TEXT("Still connected after ERROR"), Bridge->IsBridgeConnected());

    // Disconnect() stops everything and fires OnBridgeDisconnected.
    FString DisconnectReason;
    Bridge->OnBridgeDisconnectedNative.AddLambda([&DisconnectReason](const FString& Reason) { DisconnectReason = Reason; });
    Bridge->Disconnect();
    TestEqual(TEXT("Disabled after Disconnect"), Bridge->GetBridgeStatus(), EUBotROSBridgeStatus::Disabled);
    TestFalse(TEXT("Disconnect reason reported"), DisconnectReason.IsEmpty());
    TestTrue(TEXT("The bridge sees the connection close"), PumpUntil(Server, Bridge, [&Server]() { return !Server.HasClient(); }));
    TestFalse(TEXT("Test bridge saw only valid frames"), Server.bProtocolError);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotROSSubsystemPublishTest, "UBotROS.Subsystem.PublishFraming", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotROSSubsystemPublishTest::RunTest(const FString& Parameters)
{
    using namespace UBot::ROSProtocol;
    using namespace UBotROSSubsystemTests;

    FTestBridgeServer Server;
    if (!TestTrue(TEXT("Test bridge listens"), Server.Start()))
    {
        return false;
    }
    FScopedBridgeWorld Scope;
    UUBotROSBridgeSubsystem* Bridge = Scope.Bridge();
    if (!TestNotNull(TEXT("Bridge subsystem"), Bridge) || !ConnectBridge(*this, Server, Bridge))
    {
        return false;
    }

    const int32 StateId = Bridge->Advertise(TEXT("/ubot_test/pose"), TEXT("geometry_msgs/msg/PoseStamped"), FUBotROSQos::State());
    const int32 SensorId = Bridge->Advertise(TEXT("/ubot_test/image"), TEXT("sensor_msgs/msg/Image"), FUBotROSQos::SensorData());

    // Reliable: every message, in order, bytes unchanged.
    TArray<TArray<uint8>> Sent;
    for (uint8 Index = 0; Index < 5; ++Index)
    {
        Sent.Add(FakeCdr(24 + Index, static_cast<uint8>(10 * Index)));
        TestTrue(TEXT("Reliable publish accepted"), Bridge->Publish(StateId, TArray<uint8>(Sent.Last())));
    }
    if (TestTrue(TEXT("Five reliable PUBLISH frames"), PumpUntil(Server, Bridge, [&Server, StateId]() { return Server.PublishedBodies(static_cast<uint32>(StateId)).Num() == 5; })))
    {
        const TArray<TArray<uint8>> Bodies = Server.PublishedBodies(static_cast<uint32>(StateId));
        for (int32 Index = 0; Index < 5; ++Index)
        {
            TestTrue(*FString::Printf(TEXT("Reliable message %d intact and in order"), Index), Bodies[Index] == Sent[Index]);
        }
    }

    // Exact framing of one PUBLISH: header, u32 LE id, CDR.
    {
        const TArray<uint8> Cdr = { 0x00, 0x01, 0x00, 0x00, 0x11, 0x22 };
        const TArray<uint8> Expected = BuildIdFrame(EOp::Publish, static_cast<uint32>(StateId), Cdr);
        Bridge->Publish(StateId, TArray<uint8>(Cdr));
        if (TestTrue(TEXT("Framed PUBLISH arrives"), PumpUntil(Server, Bridge, [&Server]() { return Server.CountFrames(EOp::Publish) == 6; })))
        {
            const FWireFrame* Frame = Server.LastFrame(EOp::Publish);
            const TArray<uint8> Rebuilt = BuildFrame(EOp::Publish, Frame->Payload);
            TestTrue(TEXT("PUBLISH bytes = header + id LE + CDR"), Rebuilt == Expected);
            TestEqual(TEXT("Flags are zero"), Frame->Flags, uint16(0));
        }
    }

    // A large best_effort message (sent without coalescing) arrives intact.
    {
        const TArray<uint8> Large = FakeCdr(300 * 1024 + 3, 7);
        TestTrue(TEXT("Large publish accepted"), Bridge->Publish(SensorId, TArray<uint8>(Large)));
        if (TestTrue(TEXT("Large PUBLISH arrives"), PumpUntil(Server, Bridge, [&Server, SensorId]() { return Server.PublishedBodies(static_cast<uint32>(SensorId)).Num() == 1; })))
        {
            TestTrue(TEXT("Large message intact"), Server.PublishedBodies(static_cast<uint32>(SensorId))[0] == Large);
        }
    }

    // Refusals.
    AddExpectedErrorPlain(TEXT("shorter than the CDR encapsulation header"), EAutomationExpectedErrorFlags::Contains, 1);
    TestFalse(TEXT("Unknown publisher id"), Bridge->Publish(StateId + 100, FakeCdr(8, 0)));
    TestFalse(TEXT("Message without encapsulation header"), Bridge->Publish(StateId, TArray<uint8>({ 1, 2, 3 })));
    Bridge->Unadvertise(SensorId);
    TestFalse(TEXT("Unadvertised id"), Bridge->Publish(SensorId, FakeCdr(8, 0)));

    // /tf: TFMessage on the shared /tf publisher (reliable / volatile / 100).
    Bridge->PublishTransforms({ MakeTestTransform(TEXT("odom"), TEXT("base_link"), 1.0, 2.0, 0.0) });
    FTopicRequest TfRequest;
    if (TestTrue(TEXT("/tf advertised"), PumpUntil(Server, Bridge, [&]() { return Server.FindTopicRequest(EOp::Advertise, TEXT("/tf"), TfRequest); })))
    {
        TestEqualSensitive(TEXT("/tf type"), TfRequest.Type, TEXT("tf2_msgs/msg/TFMessage"));
        TestTrue(TEXT("/tf qos"), TfRequest.Qos == FUBotROSQos::Tf());
        if (TestTrue(TEXT("/tf message arrives"), PumpUntil(Server, Bridge, [&]() { return Server.PublishedBodies(TfRequest.Id).Num() == 1; })))
        {
            FUBotRosTfMessage Decoded;
            if (TestTrue(TEXT("/tf decodes"), UBot::Ros::Decode(Server.PublishedBodies(TfRequest.Id)[0], Decoded)))
            {
                TestEqual(TEXT("/tf transforms"), Decoded.transforms.Num(), 1);
                if (Decoded.transforms.Num() == 1)
                {
                    TestEqualSensitive(TEXT("/tf child"), Decoded.transforms[0].child_frame_id, TEXT("base_link"));
                    TestEqual(TEXT("/tf x"), Decoded.transforms[0].transform.translation.x, 1.0);
                }
            }
        }
    }

    const FUBotROSBridgeStats Stats = Bridge->GetStats();
    TestTrue(TEXT("Frames were sent"), Stats.FramesSent >= 8);
    TestEqual(TEXT("Nothing dropped"), Stats.DroppedMessages, int64(0));
    TestFalse(TEXT("Test bridge saw only valid frames"), Server.bProtocolError);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotROSSubsystemSubCountTest, "UBotROS.Subsystem.SubscriberCount", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotROSSubsystemSubCountTest::RunTest(const FString& Parameters)
{
    using namespace UBot::ROSProtocol;
    using namespace UBotROSSubsystemTests;

    FTestBridgeServer Server;
    if (!TestTrue(TEXT("Test bridge listens"), Server.Start()))
    {
        return false;
    }
    FScopedBridgeWorld Scope;
    UUBotROSBridgeSubsystem* Bridge = Scope.Bridge();
    if (!TestNotNull(TEXT("Bridge subsystem"), Bridge))
    {
        return false;
    }

    const int32 ImuId = Bridge->Advertise(TEXT("/ubot_test/imu"), TEXT("sensor_msgs/msg/Imu"), FUBotROSQos::SensorData());
    const int32 ScanId = Bridge->Advertise(TEXT("/ubot_test/scan"), TEXT("sensor_msgs/msg/LaserScan"), FUBotROSQos::SensorData());
    TestEqual(TEXT("Unknown before any report"), Bridge->GetSubscriptionCount(ImuId), -1);
    TestEqual(TEXT("Unknown id"), Bridge->GetSubscriptionCount(12345), -1);
    if (!ConnectBridge(*this, Server, Bridge))
    {
        return false;
    }
    TestEqual(TEXT("Still unknown after connecting"), Bridge->GetSubscriptionCount(ImuId), -1);

    // Full report; ids the client does not know are ignored.
    Server.Send(BuildJsonFrame(EOp::SubCount, MakeSubCountJson({ TPair<uint32, int32>(ImuId, 3), TPair<uint32, int32>(ScanId, 0), TPair<uint32, int32>(999, 5) })));
    TestTrue(TEXT("SUB_COUNT applied"), PumpUntil(Server, Bridge, [Bridge, ImuId]() { return Bridge->GetSubscriptionCount(ImuId) == 3; }));
    TestEqual(TEXT("Zero subscribers reported"), Bridge->GetSubscriptionCount(ScanId), 0);

    // Partial report: merged into the known counts.
    Server.Send(BuildJsonFrame(EOp::SubCount, MakeSubCountJson({ TPair<uint32, int32>(ScanId, 2) })));
    TestTrue(TEXT("Partial SUB_COUNT applied"), PumpUntil(Server, Bridge, [Bridge, ScanId]() { return Bridge->GetSubscriptionCount(ScanId) == 2; }));
    TestEqual(TEXT("Other counts kept"), Bridge->GetSubscriptionCount(ImuId), 3);

    // Malformed reports are ignored without dropping the connection.
    Server.Send(BuildJsonFrame(EOp::SubCount, TEXT("{\"counts\":{\"1\":\"many\"}}")));
    PumpFor(Server, Bridge, 0.1);
    TestEqual(TEXT("Bad entry ignored"), Bridge->GetSubscriptionCount(ImuId), 3);
    TestTrue(TEXT("Still connected"), Bridge->IsBridgeConnected());

    // Counts are unknown again once the connection is gone.
    Bridge->Disconnect();
    TestEqual(TEXT("Unknown after disconnect"), Bridge->GetSubscriptionCount(ImuId), -1);
    TestEqual(TEXT("Unknown after disconnect (scan)"), Bridge->GetSubscriptionCount(ScanId), -1);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotROSSubsystemMessageDispatchTest, "UBotROS.Subsystem.MessageDispatch", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotROSSubsystemMessageDispatchTest::RunTest(const FString& Parameters)
{
    using namespace UBot::ROSProtocol;
    using namespace UBotROSSubsystemTests;

    FTestBridgeServer Server;
    if (!TestTrue(TEXT("Test bridge listens"), Server.Start()))
    {
        return false;
    }
    FScopedBridgeWorld Scope;
    UUBotROSBridgeSubsystem* Bridge = Scope.Bridge();
    if (!TestNotNull(TEXT("Bridge subsystem"), Bridge))
    {
        return false;
    }

    struct FReceived
    {
        int32 Count = 0;
        bool bAlwaysGameThread = true;
        bool bAlwaysInsideTick = true;
        TArray<uint8> LastCdr;
    };
    FReceived Received;
    bool bInsideTick = false;

    const int32 CmdId = Bridge->Subscribe(TEXT("/ubot_test/cmd_vel"), TEXT("geometry_msgs/msg/Twist"), FUBotROSQos::SensorData(),
        [&Received, &bInsideTick](TConstArrayView<uint8> Cdr)
        {
            ++Received.Count;
            Received.bAlwaysGameThread &= IsInGameThread();
            Received.bAlwaysInsideTick &= bInsideTick;
            Received.LastCdr = TArray<uint8>(Cdr);
        });
    if (!ConnectBridge(*this, Server, Bridge))
    {
        return false;
    }

    // A real Twist (linear 1.5, angular z 0.25) as the bridge would forward it.
    FUBotRosTwist Twist;
    Twist.linear.x = 1.5;
    Twist.angular.z = 0.25;
    const TArray<uint8> TwistCdr = UBot::Ros::Encode(Twist);
    Server.Send(BuildIdFrame(EOp::Message, static_cast<uint32>(CmdId), TwistCdr));

    // Received by the I/O thread, but not dispatched until the subsystem ticks.
    TestTrue(TEXT("MESSAGE received in the background"), PumpUntil(Server, Bridge, [Bridge]() { return Bridge->GetStats().PendingIncomingMessages > 0; }, false));
    TestEqual(TEXT("No dispatch outside Tick"), Received.Count, 0);

    bInsideTick = true;
    Bridge->Tick(TickSeconds);
    bInsideTick = false;
    TestEqual(TEXT("Dispatched during Tick"), Received.Count, 1);
    TestTrue(TEXT("Callback runs on the game thread"), Received.bAlwaysGameThread);
    TestTrue(TEXT("Callback runs inside Tick"), Received.bAlwaysInsideTick);
    TestTrue(TEXT("Callback gets the CDR bytes without the id"), Received.LastCdr == TwistCdr);
    FUBotRosTwist Decoded;
    if (TestTrue(TEXT("Twist decodes"), UBot::Ros::Decode(Received.LastCdr, Decoded)))
    {
        TestEqual(TEXT("linear.x"), Decoded.linear.x, 1.5);
        TestEqual(TEXT("angular.z"), Decoded.angular.z, 0.25);
    }
    TestEqual(TEXT("Nothing pending"), Bridge->GetStats().PendingIncomingMessages, 0);

    // Several messages in one burst are dispatched in order; unknown subscription ids are dropped.
    TArray<uint8> Burst;
    Burst.Append(BuildIdFrame(EOp::Message, 777u, FakeCdr(8, 1)));
    Burst.Append(BuildIdFrame(EOp::Message, static_cast<uint32>(CmdId), FakeCdr(8, 2)));
    Burst.Append(BuildIdFrame(EOp::Message, static_cast<uint32>(CmdId), FakeCdr(8, 3)));
    Server.Send(Burst);
    TestTrue(TEXT("Burst received"), PumpUntil(Server, Bridge, [Bridge]() { return Bridge->GetStats().PendingIncomingMessages >= 3; }, false));
    bInsideTick = true;
    Bridge->Tick(TickSeconds);
    bInsideTick = false;
    TestEqual(TEXT("Two more dispatches"), Received.Count, 3);
    TestTrue(TEXT("Last message of the burst is last"), Received.LastCdr == FakeCdr(8, 3));

    // A callback may unsubscribe itself; later messages for it are discarded.
    int32 SelfCalls = 0;
    int32 SelfId = 0;
    SelfId = Bridge->Subscribe(TEXT("/ubot_test/once"), TEXT("std_msgs/msg/String"), FUBotROSQos::State(),
        [Bridge, &SelfCalls, &SelfId](TConstArrayView<uint8>)
        {
            ++SelfCalls;
            Bridge->Unsubscribe(SelfId);
        });
    TestTrue(TEXT("SUBSCRIBE sent while connected"), PumpUntil(Server, Bridge, [&Server]() { return Server.CountFrames(EOp::Subscribe) >= 2; }));
    TArray<uint8> Twice;
    Twice.Append(BuildIdFrame(EOp::Message, static_cast<uint32>(SelfId), FakeCdr(8, 4)));
    Twice.Append(BuildIdFrame(EOp::Message, static_cast<uint32>(SelfId), FakeCdr(8, 5)));
    Server.Send(Twice);
    TestTrue(TEXT("Both received"), PumpUntil(Server, Bridge, [Bridge]() { return Bridge->GetStats().PendingIncomingMessages >= 2; }, false));
    Bridge->Tick(TickSeconds);
    TestEqual(TEXT("Self-unsubscribing callback ran once"), SelfCalls, 1);
    TestFalse(TEXT("Self-unsubscribed"), Bridge->IsSubscribed(SelfId));
    TestTrue(TEXT("UNSUBSCRIBE sent"), PumpUntil(Server, Bridge, [&Server]() { return Server.CountFrames(EOp::Unsubscribe) == 1; }));
    TestEqual(TEXT("Other subscription unaffected"), Received.Count, 3);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotROSSubsystemReconnectTest, "UBotROS.Subsystem.Reconnect", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotROSSubsystemReconnectTest::RunTest(const FString& Parameters)
{
    using namespace UBot::ROSProtocol;
    using namespace UBotROSSubsystemTests;

    FTestBridgeServer Server;
    if (!TestTrue(TEXT("Test bridge listens"), Server.Start()))
    {
        return false;
    }
    // A 1 s reconnect delay leaves the test a wide window in which no session is active.
    FScopedBridgeWorld Scope(false, 268435456, 67108864, 1.0);
    UUBotROSBridgeSubsystem* Bridge = Scope.Bridge();
    if (!TestNotNull(TEXT("Bridge subsystem"), Bridge))
    {
        return false;
    }

    const int32 StateId = Bridge->Advertise(TEXT("/ubot_test/odom"), TEXT("nav_msgs/msg/Odometry"), FUBotROSQos::State());
    const int32 SensorId = Bridge->Advertise(TEXT("/ubot_test/imu"), TEXT("sensor_msgs/msg/Imu"), FUBotROSQos::SensorData());
    const int32 TempId = Bridge->Advertise(TEXT("/ubot_test/temp"), TEXT("sensor_msgs/msg/Temperature"), FUBotROSQos::State());
    const int32 CmdId = Bridge->Subscribe(TEXT("/ubot_test/cmd_vel"), TEXT("geometry_msgs/msg/Twist"), FUBotROSQos::SensorData(), [](TConstArrayView<uint8>) {});

    int32 ConnectedEvents = 0;
    TArray<FString> DisconnectReasons;
    Bridge->OnBridgeConnectedNative.AddLambda([&ConnectedEvents]() { ++ConnectedEvents; });
    Bridge->OnBridgeDisconnectedNative.AddLambda([&DisconnectReasons](const FString& Reason) { DisconnectReasons.Add(Reason); });

    if (!ConnectBridge(*this, Server, Bridge))
    {
        return false;
    }
    TestTrue(TEXT("First session registered everything"), PumpUntil(Server, Bridge, [&Server]() { return Server.CountFrames(EOp::Advertise) == 3 && Server.CountFrames(EOp::Subscribe) == 1; }));

    // The bridge goes away.
    AddExpectedErrorPlain(TEXT("lost the connection"), EAutomationExpectedErrorFlags::Contains, 1);
    Server.CloseClient();
    if (!TestTrue(TEXT("Connection loss detected"), PumpUntil(Server, Bridge, [Bridge]() { return Bridge->GetBridgeStatus() == EUBotROSBridgeStatus::Error; }, true)))
    {
        return false;
    }
    TestEqual(TEXT("OnBridgeDisconnected fired once"), DisconnectReasons.Num(), 1);
    TestTrue(TEXT("Disconnect reason"), DisconnectReasons.Num() == 1 && !DisconnectReasons[0].IsEmpty());
    TestFalse(TEXT("Last error set"), Bridge->GetLastBridgeError().IsEmpty());

    // While reconnecting: reliable data is queued, best_effort data is dropped, unadvertised publishers vanish.
    const FUBotROSBridgeStats Before = Bridge->GetStats();
    const TArray<uint8> Queued = FakeCdr(32, 42);
    TestTrue(TEXT("Reliable publish while disconnected"), Bridge->Publish(StateId, TArray<uint8>(Queued)));
    TestTrue(TEXT("best_effort publish while disconnected"), Bridge->Publish(SensorId, FakeCdr(32, 9)));
    TestTrue(TEXT("Reliable publish of a publisher about to go"), Bridge->Publish(TempId, FakeCdr(16, 1)));
    Bridge->Unadvertise(TempId);
    const FUBotROSBridgeStats During = Bridge->GetStats();
    TestEqual(TEXT("best_effort and orphaned reliable data dropped"), During.DroppedMessages - Before.DroppedMessages, int64(2));

    // Reconnect: HELLO, the same ids again, then the queued reliable message.
    if (!TestTrue(TEXT("Reconnected"), PumpUntil(Server, Bridge, [Bridge]() { return Bridge->IsBridgeConnected(); })))
    {
        return false;
    }
    TestEqual(TEXT("Second TCP connection"), Server.AcceptCount, 2);
    TestEqual(TEXT("OnBridgeConnected fired again"), ConnectedEvents, 2);
    TestTrue(TEXT("Session count"), Bridge->GetStats().SessionCount == 2);
    TestTrue(TEXT("Second session registered and flushed"), PumpUntil(Server, Bridge, [&Server, StateId]() { return Server.CountFrames(EOp::Subscribe) == 1 && Server.PublishedBodies(static_cast<uint32>(StateId)).Num() == 1; }));

    if (TestTrue(TEXT("Frames of the second session"), Server.Frames.Num() >= 4))
    {
        TestEqual(TEXT("HELLO first again"), Server.Frames[0].Op, static_cast<uint8>(EOp::Hello));
        TestEqual(TEXT("Re-advertised publishers"), Server.CountFrames(EOp::Advertise), 2);
        FTopicRequest Request;
        TestTrue(TEXT("Odom re-advertised with the same id"), Server.FindTopicRequest(EOp::Advertise, TEXT("/ubot_test/odom"), Request) && Request.Id == static_cast<uint32>(StateId));
        TestTrue(TEXT("IMU re-advertised with the same id"), Server.FindTopicRequest(EOp::Advertise, TEXT("/ubot_test/imu"), Request) && Request.Id == static_cast<uint32>(SensorId));
        TestFalse(TEXT("Unadvertised publisher not re-advertised"), Server.FindTopicRequest(EOp::Advertise, TEXT("/ubot_test/temp"), Request));
        TestTrue(TEXT("Subscription re-sent with the same id"), Server.FindTopicRequest(EOp::Subscribe, TEXT("/ubot_test/cmd_vel"), Request) && Request.Id == static_cast<uint32>(CmdId));

        // Registrations precede the queued data.
        int32 LastRegistration = -1;
        int32 FirstPublish = MAX_int32;
        for (int32 Index = 0; Index < Server.Frames.Num(); ++Index)
        {
            const uint8 Op = Server.Frames[Index].Op;
            if (Op == static_cast<uint8>(EOp::Advertise) || Op == static_cast<uint8>(EOp::Subscribe))
            {
                LastRegistration = Index;
            }
            if (Op == static_cast<uint8>(EOp::Publish))
            {
                FirstPublish = FMath::Min(FirstPublish, Index);
            }
        }
        TestTrue(TEXT("ADVERTISE/SUBSCRIBE before the queued PUBLISH"), LastRegistration < FirstPublish);
        TestTrue(TEXT("Queued reliable message delivered intact"), Server.PublishedBodies(static_cast<uint32>(StateId))[0] == Queued);
        TestEqual(TEXT("Dropped best_effort message not delivered"), Server.PublishedBodies(static_cast<uint32>(SensorId)).Num(), 0);
        TestEqual(TEXT("Orphaned message not delivered"), Server.PublishedBodies(static_cast<uint32>(TempId)).Num(), 0);
    }
    TestFalse(TEXT("Test bridge saw only valid frames"), Server.bProtocolError);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotROSSubsystemStaticTfTest, "UBotROS.Subsystem.StaticTfAggregation", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotROSSubsystemStaticTfTest::RunTest(const FString& Parameters)
{
    using namespace UBot::ROSProtocol;
    using namespace UBotROSSubsystemTests;

    FTestBridgeServer Server;
    if (!TestTrue(TEXT("Test bridge listens"), Server.Start()))
    {
        return false;
    }
    FScopedBridgeWorld Scope;
    UUBotROSBridgeSubsystem* Bridge = Scope.Bridge();
    if (!TestNotNull(TEXT("Bridge subsystem"), Bridge) || !ConnectBridge(*this, Server, Bridge))
    {
        return false;
    }

    AActor* OwnerA = Scope.Get()->SpawnActor<AActor>();
    AActor* OwnerB = Scope.Get()->SpawnActor<AActor>();
    if (!TestNotNull(TEXT("Owner A"), OwnerA) || !TestNotNull(TEXT("Owner B"), OwnerB))
    {
        return false;
    }

    // Two owners in the same frame: one merged message.
    const FUBotRosTransformStamped ImuTf = MakeTestTransform(TEXT("base_link"), TEXT("imu"), 1.0, 2.0, 3.0);
    const FUBotRosTransformStamped CamTf = MakeTestTransform(TEXT("base_link"), TEXT("cam"), 0.5, 0.0, 0.2);
    const FUBotRosTransformStamped OpticalTf = MakeTestTransform(TEXT("cam"), TEXT("cam_optical"), 0.0, 0.0, 0.0);
    Bridge->SetStaticTransforms(OwnerA, { ImuTf });
    Bridge->SetStaticTransforms(OwnerB, { CamTf, OpticalTf });

    FTopicRequest Request;
    if (!TestTrue(TEXT("/tf_static advertised"), PumpUntil(Server, Bridge, [&]() { return Server.FindTopicRequest(EOp::Advertise, TEXT("/tf_static"), Request); })))
    {
        return false;
    }
    TestEqualSensitive(TEXT("/tf_static type"), Request.Type, TEXT("tf2_msgs/msg/TFMessage"));
    TestTrue(TEXT("/tf_static latched qos (reliable / transient_local / 1)"), Request.Qos == FUBotROSQos::Latched());
    const uint32 StaticId = Request.Id;

    TestTrue(TEXT("Merged message arrives"), PumpUntil(Server, Bridge, [&]() { return Server.PublishedBodies(StaticId).Num() >= 1; }));
    PumpFor(Server, Bridge, 0.2);
    TArray<TArray<uint8>> Bodies = Server.PublishedBodies(StaticId);
    TestEqual(TEXT("Exactly one /tf_static message for both owners"), Bodies.Num(), 1);
    if (Bodies.Num() >= 1)
    {
        FUBotRosTfMessage Expected;
        Expected.transforms = { ImuTf, CamTf, OpticalTf };
        TestTrue(TEXT("Merged message = owners in registration order"), Bodies[0] == UBot::Ros::Encode(Expected));
        FUBotRosTfMessage Decoded;
        if (TestTrue(TEXT("Merged message decodes"), UBot::Ros::Decode(Bodies[0], Decoded)))
        {
            TestEqualSensitive(TEXT("Children"), FString::Join(ChildFrames(Decoded), TEXT(",")), TEXT("imu,cam,cam_optical"));
        }
    }
    TestEqual(TEXT("GetMergedStaticTransforms"), Bridge->GetMergedStaticTransforms().Num(), 3);

    // Setting the same transforms again changes nothing: no new message.
    Bridge->SetStaticTransforms(OwnerA, { ImuTf });
    PumpFor(Server, Bridge, 0.2);
    TestEqual(TEXT("Unchanged set is not re-published"), Server.PublishedBodies(StaticId).Num(), 1);

    // Clearing one owner re-publishes the rest.
    Bridge->ClearStaticTransforms(OwnerA);
    TestTrue(TEXT("Re-published after a clear"), PumpUntil(Server, Bridge, [&]() { return Server.PublishedBodies(StaticId).Num() == 2; }));
    Bodies = Server.PublishedBodies(StaticId);
    if (Bodies.Num() == 2)
    {
        FUBotRosTfMessage Decoded;
        if (TestTrue(TEXT("Second message decodes"), UBot::Ros::Decode(Bodies[1], Decoded)))
        {
            TestEqualSensitive(TEXT("Only owner B left"), FString::Join(ChildFrames(Decoded), TEXT(",")), TEXT("cam,cam_optical"));
        }
    }

    // The same child frame from two owners: the owner registered last wins (with a warning).
    AddExpectedErrorPlain(TEXT("is set by several owners"), EAutomationExpectedErrorFlags::Contains, 1);
    Bridge->SetStaticTransforms(OwnerA, { MakeTestTransform(TEXT("base_link"), TEXT("cam"), 9.0, 9.0, 9.0) });
    TestTrue(TEXT("Re-published after the conflict"), PumpUntil(Server, Bridge, [&]() { return Server.PublishedBodies(StaticId).Num() == 3; }));
    Bodies = Server.PublishedBodies(StaticId);
    if (Bodies.Num() == 3)
    {
        FUBotRosTfMessage Decoded;
        if (TestTrue(TEXT("Third message decodes"), UBot::Ros::Decode(Bodies[2], Decoded)) && TestEqual(TEXT("Still two frames"), Decoded.transforms.Num(), 2))
        {
            TestEqualSensitive(TEXT("Frame order kept"), FString::Join(ChildFrames(Decoded), TEXT(",")), TEXT("cam,cam_optical"));
            TestEqual(TEXT("Later owner wins"), Decoded.transforms[0].transform.translation.x, 9.0);
        }
    }

    // A reconnect re-sends the latched message (the bridge's publisher may be new).
    AddExpectedErrorPlain(TEXT("lost the connection"), EAutomationExpectedErrorFlags::Contains, 1);
    Server.CloseClient();
    TestTrue(TEXT("Reconnected"), PumpUntil(Server, Bridge, [&Server, Bridge]() { return Server.AcceptCount == 2 && Bridge->IsBridgeConnected(); }));
    TestTrue(TEXT("Latched message re-sent after reconnect"), PumpUntil(Server, Bridge, [&]() { return Server.PublishedBodies(StaticId).Num() == 1; }));
    TestTrue(TEXT("/tf_static re-advertised"), Server.FindTopicRequest(EOp::Advertise, TEXT("/tf_static"), Request) && Request.Id == StaticId);
    Bodies = Server.PublishedBodies(StaticId);
    if (Bodies.Num() == 1)
    {
        FUBotRosTfMessage Decoded;
        TestTrue(TEXT("Re-sent message is the current set"), UBot::Ros::Decode(Bodies[0], Decoded) && Decoded.transforms.Num() == 2);
    }

    // Owners destroyed without clearing are pruned; an empty set is published once.
    OwnerA->Destroy();
    OwnerB->Destroy();
    TestTrue(TEXT("Empty set published after the owners died"), PumpUntil(Server, Bridge, [&]() { return Server.PublishedBodies(StaticId).Num() == 2; }));
    Bodies = Server.PublishedBodies(StaticId);
    if (Bodies.Num() == 2)
    {
        FUBotRosTfMessage Decoded;
        TestTrue(TEXT("Empty TFMessage"), UBot::Ros::Decode(Bodies[1], Decoded) && Decoded.transforms.Num() == 0);
    }
    TestEqual(TEXT("Nothing merged"), Bridge->GetMergedStaticTransforms().Num(), 0);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotROSSubsystemQueuePolicyTest, "UBotROS.Subsystem.QueuePolicy", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotROSSubsystemQueuePolicyTest::RunTest(const FString& Parameters)
{
    using namespace UBot::ROSProtocol;
    using namespace UBotROSSubsystemTests;

    FTestBridgeServer Server;
    if (!TestTrue(TEXT("Test bridge listens"), Server.Start()))
    {
        return false;
    }
    // Small budgets so the limits are reached with a few messages.
    FScopedBridgeWorld Scope(false, 4096, 2048);
    UUBotROSBridgeSubsystem* Bridge = Scope.Bridge();
    if (!TestNotNull(TEXT("Bridge subsystem"), Bridge))
    {
        return false;
    }

    // Disabled (never connected): nothing is queued, publishing still succeeds.
    const int32 SensorId = Bridge->Advertise(TEXT("/ubot_test/scan"), TEXT("sensor_msgs/msg/LaserScan"), FUBotROSQos::SensorData());
    const int32 StateId = Bridge->Advertise(TEXT("/ubot_test/odom"), TEXT("nav_msgs/msg/Odometry"), FUBotROSQos::State());
    TestTrue(TEXT("Publish while disabled"), Bridge->Publish(StateId, FakeCdr(64, 0)));
    TestEqual(TEXT("Disabled: dropped"), Bridge->GetStats().DroppedMessages, int64(1));
    TestEqual(TEXT("Disabled: nothing queued"), Bridge->GetStats().QueuedBytes, int64(0));

    if (!ConnectBridge(*this, Server, Bridge))
    {
        return false;
    }
    TestTrue(TEXT("Registrations sent"), PumpUntil(Server, Bridge, [&Server]() { return Server.CountFrames(EOp::Advertise) == 2; }));

    // best_effort keeps only the newest pending message.
    Bridge->SetSendingPaused(true);
    const int64 DroppedBefore = Bridge->GetStats().DroppedMessages;
    TestTrue(TEXT("best_effort 1"), Bridge->Publish(SensorId, FakeCdr(100, 1)));
    TestTrue(TEXT("best_effort 2"), Bridge->Publish(SensorId, FakeCdr(100, 2)));
    TestTrue(TEXT("best_effort 3"), Bridge->Publish(SensorId, FakeCdr(100, 3)));
    TestEqual(TEXT("Two replaced messages counted as dropped"), Bridge->GetStats().DroppedMessages - DroppedBefore, int64(2));
    Bridge->SetSendingPaused(false);
    TestTrue(TEXT("Newest best_effort message delivered"), PumpUntil(Server, Bridge, [&Server, SensorId]() { return Server.PublishedBodies(static_cast<uint32>(SensorId)).Num() >= 1; }));
    PumpFor(Server, Bridge, 0.1);
    TArray<TArray<uint8>> SensorBodies = Server.PublishedBodies(static_cast<uint32>(SensorId));
    TestEqual(TEXT("Exactly one best_effort message"), SensorBodies.Num(), 1);
    TestTrue(TEXT("It is the newest"), SensorBodies.Num() == 1 && SensorBodies[0] == FakeCdr(100, 3));

    // Budget 4096 B: best_effort 1016 + reliable 1816 fit; the next reliable evicts the best_effort
    // message; the one after that does not fit and is rejected.
    AddExpectedErrorPlain(TEXT("uBot ROS bridge: rejected"), EAutomationExpectedErrorFlags::Contains, 0);
    Bridge->SetSendingPaused(true);
    const FUBotROSBridgeStats Before = Bridge->GetStats();
    TestTrue(TEXT("best_effort queued"), Bridge->Publish(SensorId, FakeCdr(1000, 50)));
    TestTrue(TEXT("Reliable 1 queued"), Bridge->Publish(StateId, FakeCdr(1800, 60)));
    TestTrue(TEXT("Reliable 2 queued (evicts best_effort)"), Bridge->Publish(StateId, FakeCdr(1800, 70)));
    TestFalse(TEXT("Reliable 3 rejected (queue full)"), Bridge->Publish(StateId, FakeCdr(1800, 80)));
    TestFalse(TEXT("Message above MaxPayloadBytes rejected"), Bridge->Publish(StateId, FakeCdr(2045, 90)));
    const FUBotROSBridgeStats After = Bridge->GetStats();
    TestEqual(TEXT("Evicted best_effort counted as dropped"), After.DroppedMessages - Before.DroppedMessages, int64(1));
    TestEqual(TEXT("Two rejections"), After.RejectedMessages - Before.RejectedMessages, int64(2));
    TestTrue(TEXT("Queue within budget"), After.QueuedBytes <= 4096);

    const int32 StateBefore = Server.PublishedBodies(static_cast<uint32>(StateId)).Num();
    Bridge->SetSendingPaused(false);
    TestTrue(TEXT("Queued reliable messages delivered"), PumpUntil(Server, Bridge, [&Server, StateId, StateBefore]() { return Server.PublishedBodies(static_cast<uint32>(StateId)).Num() == StateBefore + 2; }));
    PumpFor(Server, Bridge, 0.1);
    const TArray<TArray<uint8>> StateBodies = Server.PublishedBodies(static_cast<uint32>(StateId));
    TestEqual(TEXT("Only the two accepted reliable messages"), StateBodies.Num(), StateBefore + 2);
    if (StateBodies.Num() == StateBefore + 2)
    {
        TestTrue(TEXT("Reliable 1 first"), StateBodies[StateBefore] == FakeCdr(1800, 60));
        TestTrue(TEXT("Reliable 2 second"), StateBodies[StateBefore + 1] == FakeCdr(1800, 70));
    }
    TestEqual(TEXT("Evicted best_effort message never sent"), Server.PublishedBodies(static_cast<uint32>(SensorId)).Num(), 1);
    TestTrue(TEXT("Queue drained"), PumpUntil(Server, Bridge, [Bridge]() { return Bridge->GetStats().QueuedBytes == 0; }));

    // Disconnect() discards and Disabled drops again.
    Bridge->Disconnect();
    const int64 DroppedAtDisable = Bridge->GetStats().DroppedMessages;
    TestTrue(TEXT("Publish after Disconnect"), Bridge->Publish(StateId, FakeCdr(64, 1)));
    TestEqual(TEXT("Dropped while disabled"), Bridge->GetStats().DroppedMessages - DroppedAtDisable, int64(1));
    TestEqual(TEXT("Nothing queued while disabled"), Bridge->GetStats().QueuedBytes, int64(0));
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotROSSubsystemClockTest, "UBotROS.Subsystem.Clock", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotROSSubsystemClockTest::RunTest(const FString& Parameters)
{
    using namespace UBot::ROSProtocol;
    using namespace UBotROSSubsystemTests;

    FTestBridgeServer Server;
    if (!TestTrue(TEXT("Test bridge listens"), Server.Start()))
    {
        return false;
    }
    FScopedBridgeWorld Scope(true);
    UUBotROSBridgeSubsystem* Bridge = Scope.Bridge();
    if (!TestNotNull(TEXT("Bridge subsystem"), Bridge))
    {
        return false;
    }

    // Nothing is advertised for /clock before the bridge is connected.
    Bridge->Tick(TickSeconds);
    if (!ConnectBridge(*this, Server, Bridge))
    {
        return false;
    }

    FTopicRequest Request;
    if (!TestTrue(TEXT("/clock advertised"), PumpUntil(Server, Bridge, [&]() { return Server.FindTopicRequest(EOp::Advertise, TEXT("/clock"), Request); })))
    {
        return false;
    }
    TestEqualSensitive(TEXT("/clock type"), Request.Type, TEXT("rosgraph_msgs/msg/Clock"));
    TestTrue(TEXT("/clock qos reliable / volatile / 10"), Request.Qos == FUBotROSQos(EUBotROSReliability::Reliable, EUBotROSDurability::Volatile, 10));
    TestEqual(TEXT("Advertised once"), Server.CountFrames(EOp::Advertise), 1);

    // One message per tick with the world's sim time.
    TestTrue(TEXT("Clock messages arrive"), PumpUntil(Server, Bridge, [&]() { return Server.PublishedBodies(Request.Id).Num() >= 3; }));
    const TArray<TArray<uint8>> Bodies = Server.PublishedBodies(Request.Id);
    FUBotRosClock Clock;
    if (Bodies.Num() > 0 && TestTrue(TEXT("Clock decodes"), UBot::Ros::Decode(Bodies.Last(), Clock)))
    {
        int32 ExpectedSec = 0;
        uint32 ExpectedNanosec = 0;
        UBot::Ros::SecondsToRosTime(Scope.Get()->GetTimeSeconds(), ExpectedSec, ExpectedNanosec);
        TestEqual(TEXT("clock.sec"), Clock.clock.sec, ExpectedSec);
        TestEqual(TEXT("clock.nanosec"), Clock.clock.nanosec, ExpectedNanosec);
    }

    // Turning the setting off unadvertises the clock.
    GetMutableDefault<UUBotROSSettings>()->bPublishClock = false;
    TestTrue(TEXT("/clock unadvertised"), PumpUntil(Server, Bridge, [&Server]() { return Server.CountFrames(EOp::Unadvertise) == 1; }));
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotROSSubsystemFramesBeforeMalformedHeaderTest, "UBotROS.Subsystem.FramesBeforeMalformedHeader", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotROSSubsystemFramesBeforeMalformedHeaderTest::RunTest(const FString& Parameters)
{
    using namespace UBot::ROSProtocol;
    using namespace UBotROSSubsystemTests;

    FTestBridgeServer Server;
    if (!TestTrue(TEXT("Test bridge listens"), Server.Start()))
    {
        return false;
    }
    // A long reconnect delay keeps the failed session observable (no immediate new connection).
    FScopedBridgeWorld Scope(false, 268435456, 67108864, 5.0);
    UUBotROSBridgeSubsystem* Bridge = Scope.Bridge();
    if (!TestNotNull(TEXT("Bridge subsystem"), Bridge))
    {
        return false;
    }

    TArray<TArray<uint8>> Received;
    const int32 CmdId = Bridge->Subscribe(TEXT("/ubot_test/cmd_vel"), TEXT("geometry_msgs/msg/Twist"), FUBotROSQos::SensorData(),
        [&Received](TConstArrayView<uint8> Cdr) { Received.Emplace(Cdr); });
    FString DisconnectReason;
    Bridge->OnBridgeDisconnectedNative.AddLambda([&DisconnectReason](const FString& Reason) { DisconnectReason = Reason; });
    if (!ConnectBridge(*this, Server, Bridge))
    {
        return false;
    }

    // One read delivers a MESSAGE, an ERROR and then a header with a bad magic. PROTOCOL.md section 2: frames
    // completely received before the malformed header are still processed.
    FUBotRosTwist Twist;
    Twist.linear.x = 1.5;
    Twist.angular.z = 0.25;
    const TArray<uint8> TwistCdr = UBot::Ros::Encode(Twist);
    TArray<uint8> Stream = BuildIdFrame(EOp::Message, static_cast<uint32>(CmdId), TwistCdr);
    Stream.Append(BuildJsonFrame(EOp::Error, MakeErrorJson(TOptional<uint32>(), TEXT("qos_conflict"), TEXT("sent before the garbage"))));
    const uint8 Garbage[HeaderSize] = { 'X', 'B', 'R', 'B', 1, static_cast<uint8>(EOp::Ping), 0, 0, 0, 0, 0, 0 };
    Stream.Append(Garbage, HeaderSize);

    AddExpectedErrorPlain(TEXT("reported 'qos_conflict'"), EAutomationExpectedErrorFlags::Contains, 1);
    AddExpectedErrorPlain(TEXT("lost the connection"), EAutomationExpectedErrorFlags::Contains, 1);
    TestTrue(TEXT("Stream sent"), Server.Send(Stream));
    if (!TestTrue(TEXT("The session ends because of the malformed header"), PumpUntil(Server, Bridge, [Bridge]() { return Bridge->GetBridgeStatus() == EUBotROSBridgeStatus::Error; })))
    {
        return false;
    }

    if (TestEqual(TEXT("The MESSAGE before the garbage was delivered"), Received.Num(), 1))
    {
        TestTrue(TEXT("with its bytes intact"), Received[0] == TwistCdr);
    }
    TestTrue(TEXT("The disconnect reason names the malformed frame"), DisconnectReason.Contains(TEXT("malformed frame")));
    TestTrue(TEXT("and the cause"), DisconnectReason.Contains(TEXT("bad magic")));
    TestTrue(TEXT("Both complete frames were counted as received (plus the HELLO_ACK)"), Bridge->GetStats().FramesReceived >= 3);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotROSSubsystemSmallQueueTest, "UBotROS.Subsystem.QueueSmallerThanOneFrame", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotROSSubsystemSmallQueueTest::RunTest(const FString& Parameters)
{
    using namespace UBot::ROSProtocol;
    using namespace UBotROSSubsystemTests;

    FTestBridgeServer Server;
    if (!TestTrue(TEXT("Test bridge listens"), Server.Start()))
    {
        return false;
    }
    // MaxQueuedBytes (2048) is far below one maximum frame (8192 + 12 header bytes): the subsystem raises it
    // when connecting instead of dropping every large message in silence.
    FScopedBridgeWorld Scope(false, 2048, 8192);
    UUBotROSBridgeSubsystem* Bridge = Scope.Bridge();
    if (!TestNotNull(TEXT("Bridge subsystem"), Bridge))
    {
        return false;
    }

    TArray<TArray<uint8>> Received;
    const int32 CmdId = Bridge->Subscribe(TEXT("/ubot_test/big_cmd"), TEXT("std_msgs/msg/String"), FUBotROSQos::SensorData(),
        [&Received](TConstArrayView<uint8> Cdr) { Received.Emplace(Cdr); });
    const int32 SensorId = Bridge->Advertise(TEXT("/ubot_test/big_image"), TEXT("sensor_msgs/msg/Image"), FUBotROSQos::SensorData());
    const int32 StateId = Bridge->Advertise(TEXT("/ubot_test/big_state"), TEXT("sensor_msgs/msg/PointCloud2"), FUBotROSQos::State());

    AddExpectedErrorPlain(TEXT("MaxQueuedBytes (2048) is smaller than one maximum frame"), EAutomationExpectedErrorFlags::Contains, 1);
    if (!ConnectBridge(*this, Server, Bridge))
    {
        return false;
    }

    // Outgoing: 8000-byte messages (frame 8016 bytes) fit although they are four times the configured queue.
    const TArray<uint8> BestEffortMessage = FakeCdr(8000, 5);
    const TArray<uint8> ReliableMessage = FakeCdr(8000, 77);
    TestTrue(TEXT("Large best_effort publish accepted"), Bridge->Publish(SensorId, TArray<uint8>(BestEffortMessage)));
    if (TestTrue(TEXT("Large best_effort message arrives"), PumpUntil(Server, Bridge, [&Server, SensorId]() { return Server.PublishedBodies(static_cast<uint32>(SensorId)).Num() == 1; })))
    {
        TestTrue(TEXT("intact"), Server.PublishedBodies(static_cast<uint32>(SensorId))[0] == BestEffortMessage);
    }
    TestTrue(TEXT("Large reliable publish accepted"), Bridge->Publish(StateId, TArray<uint8>(ReliableMessage)));
    if (TestTrue(TEXT("Large reliable message arrives"), PumpUntil(Server, Bridge, [&Server, StateId]() { return Server.PublishedBodies(static_cast<uint32>(StateId)).Num() == 1; })))
    {
        TestTrue(TEXT("intact"), Server.PublishedBodies(static_cast<uint32>(StateId))[0] == ReliableMessage);
    }
    const FUBotROSBridgeStats Stats = Bridge->GetStats();
    TestEqual(TEXT("Nothing dropped silently"), Stats.DroppedMessages, int64(0));
    TestEqual(TEXT("Nothing rejected"), Stats.RejectedMessages, int64(0));

    // Incoming: the dispatch backlog budget is raised the same way.
    const TArray<uint8> Incoming = FakeCdr(8000, 9);
    Server.Send(BuildIdFrame(EOp::Message, static_cast<uint32>(CmdId), Incoming));
    if (TestTrue(TEXT("Large MESSAGE is dispatched"), PumpUntil(Server, Bridge, [&Received]() { return Received.Num() == 1; })))
    {
        TestTrue(TEXT("intact"), Received[0] == Incoming);
    }
    TestEqual(TEXT("No incoming message dropped"), Bridge->GetStats().DroppedIncomingMessages, int64(0));
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotROSSubsystemHostNameTest, "UBotROS.Subsystem.HostNameResolution", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotROSSubsystemHostNameTest::RunTest(const FString& Parameters)
{
    using namespace UBot::ROSProtocol;
    using namespace UBotROSSubsystemTests;

    FTestBridgeServer Server;
    if (!TestTrue(TEXT("Test bridge listens"), Server.Start()))
    {
        return false;
    }
    FScopedBridgeWorld Scope;
    UUBotROSBridgeSubsystem* Bridge = Scope.Bridge();
    if (!TestNotNull(TEXT("Bridge subsystem"), Bridge))
    {
        return false;
    }

    // A host name (not an IP literal) is resolved on a pool thread and then connected to.
    Bridge->ConnectTo(TEXT("localhost"), Server.GetPort());
    TestTrue(TEXT("Handshake through a resolved host name"), PumpUntil(Server, Bridge, [Bridge]() { return Bridge->IsBridgeConnected(); }));
    TestEqualSensitive(TEXT("Endpoint keeps the host name"), Bridge->GetBridgeEndpoint(), FString::Printf(TEXT("localhost:%d"), Server.GetPort()));
    Bridge->Disconnect();

    // The lookup of an unresolvable name never blocks Disconnect() or a switch to another endpoint: the I/O
    // thread only polls the lookup (it runs on a pool thread), so joining it does not wait for getaddrinfo.
    const double Start = FPlatformTime::Seconds();
    Bridge->ConnectTo(TEXT("ubot-no-such-host.invalid"), 8268);
    FPlatformProcess::Sleep(0.1f);
    Bridge->Disconnect();
    const double DisconnectSeconds = FPlatformTime::Seconds() - Start;
    TestTrue(*FString::Printf(TEXT("Disconnect() during a name lookup returns promptly (%.3f s)"), DisconnectSeconds), DisconnectSeconds < 0.6);
    TestEqual(TEXT("Disabled after Disconnect"), Bridge->GetBridgeStatus(), EUBotROSBridgeStatus::Disabled);

    const double SwitchStart = FPlatformTime::Seconds();
    Bridge->ConnectTo(TEXT("ubot-no-such-host.invalid"), 8268);
    FPlatformProcess::Sleep(0.1f);
    Bridge->ConnectTo(TEXT("127.0.0.1"), Server.GetPort());
    TestTrue(*FString::Printf(TEXT("Switching endpoints during a name lookup is prompt (%.3f s)"), FPlatformTime::Seconds() - SwitchStart), FPlatformTime::Seconds() - SwitchStart < 0.6);
    TestTrue(TEXT("Connected to the new endpoint"), PumpUntil(Server, Bridge, [Bridge]() { return Bridge->IsBridgeConnected(); }));
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotROSSubsystemRuntimeStatusTest, "UBotROS.Subsystem.RuntimeStatus", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotROSSubsystemRuntimeStatusTest::RunTest(const FString& Parameters)
{
    using namespace UBotROSSubsystemTests;

    const FString StatusId = TEXT("ros.bridge");
    const FUBotRuntimeStatus& RuntimeStatus = FUBotRuntimeStatus::Get();

    // A port nothing listens on any more.
    int32 ClosedPort = 0;
    {
        FTestBridgeServer Probe;
        if (!TestTrue(TEXT("Probe listener"), Probe.Start()))
        {
            return false;
        }
        ClosedPort = Probe.GetPort();
    }
    const FString ClosedEndpoint = FString::Printf(TEXT("127.0.0.1:%d"), ClosedPort);

    {
        FScopedBridgeWorld Scope;
        UUBotROSBridgeSubsystem* Bridge = Scope.Bridge();
        if (!TestNotNull(TEXT("Bridge subsystem"), Bridge))
        {
            return false;
        }
        TestNull(TEXT("Nothing is published before connecting"), RuntimeStatus.FindItem(StatusId));

        AddExpectedErrorPlain(TEXT("cannot reach ubot_ros_bridge"), EAutomationExpectedErrorFlags::Contains, 1);
        Bridge->ConnectTo(TEXT("127.0.0.1"), ClosedPort);
        const FUBotRuntimeItem* Item = RuntimeStatus.FindItem(StatusId);
        if (TestNotNull(TEXT("Connecting is published"), Item))
        {
            TestEqualSensitive(TEXT("Connecting value"), *Item->Value, TEXT("connecting"));
            TestEqualSensitive(TEXT("Address as detail"), *Item->Detail, *ClosedEndpoint);
            TestEqual(TEXT("Label"), Item->Label.BuildSourceString(), FString(TEXT("ROS bridge")));
        }
        TestNull(TEXT("No problem while the first attempt runs"), RuntimeStatus.FindProblem(StatusId));

        FTestBridgeServer NoServer;
        TestTrue(TEXT("The failed attempt reports rosBridgeUnreachable"),
            PumpUntil(NoServer, Bridge, [&RuntimeStatus, &StatusId]() { return RuntimeStatus.FindProblem(StatusId) != nullptr; }));
        if (const FUBotRuntimeProblem* Problem = RuntimeStatus.FindProblem(StatusId))
        {
            TestEqualSensitive(TEXT("Problem code"), *Problem->Code, TEXT("rosBridgeUnreachable"));
            TestEqualSensitive(TEXT("Problem package"), *Problem->Package, TEXT("UBotROS"));
            TestEqual(TEXT("Problem severity"), Problem->Severity, EUBotRuntimeSeverity::Info);
            TestTrue(TEXT("Address parameter"), Problem->Params.Num() == 1 && Problem->Params[0].Key == TEXT("address") && Problem->Params[0].Value == ClosedEndpoint);
            TestEqual(TEXT("Problem message"), Problem->Message.BuildSourceString(),
                FString::Printf(TEXT("Could not connect to ubot_ros_bridge at %s."), *ClosedEndpoint));
        }
        Item = RuntimeStatus.FindItem(StatusId);
        TestTrue(TEXT("Unreachable bridge is disconnected"), Item && Item->Value == TEXT("disconnected") && Item->Severity == EUBotRuntimeSeverity::Warning);

        FTestBridgeServer Server;
        if (!TestTrue(TEXT("Test bridge listens"), Server.Start()))
        {
            return false;
        }
        if (!ConnectBridge(*this, Server, Bridge))
        {
            return false;
        }
        Item = RuntimeStatus.FindItem(StatusId);
        if (TestNotNull(TEXT("Connected is published"), Item))
        {
            TestEqualSensitive(TEXT("Connected value"), *Item->Value, TEXT("connected"));
            TestEqual(TEXT("Connected severity"), Item->Severity, EUBotRuntimeSeverity::Ok);
            TestEqualSensitive(TEXT("New address as detail"), *Item->Detail, *FString::Printf(TEXT("127.0.0.1:%d"), Server.GetPort()));
        }
        TestNull(TEXT("Connecting clears the problem"), RuntimeStatus.FindProblem(StatusId));

        Bridge->Disconnect();
        Item = RuntimeStatus.FindItem(StatusId);
        TestTrue(TEXT("Disconnect() is published"), Item && Item->Value == TEXT("disconnected") && Item->Severity == EUBotRuntimeSeverity::Info);
        TestNull(TEXT("A deliberate disconnect is not a problem"), RuntimeStatus.FindProblem(StatusId));
    }

    TestNull(TEXT("The item goes away with the world"), RuntimeStatus.FindItem(StatusId));
    TestNull(TEXT("No problem outlives the world"), RuntimeStatus.FindProblem(StatusId));
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotROSSubsystemInvalidEndpointStatusTest, "UBotROS.Subsystem.InvalidEndpointStatus", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotROSSubsystemInvalidEndpointStatusTest::RunTest(const FString& Parameters)
{
    using namespace UBotROSSubsystemTests;

    const FString StatusId = TEXT("ros.bridge");
    const FUBotRuntimeStatus& RuntimeStatus = FUBotRuntimeStatus::Get();

    FTestBridgeServer Server;
    if (!TestTrue(TEXT("Test bridge listens"), Server.Start()))
    {
        return false;
    }
    FScopedBridgeWorld Scope;
    UUBotROSBridgeSubsystem* Bridge = Scope.Bridge();
    if (!TestNotNull(TEXT("Bridge subsystem"), Bridge) || !ConnectBridge(*this, Server, Bridge))
    {
        return false;
    }
    const FString PreviousEndpoint = Bridge->GetBridgeEndpoint();

    // The report about a rejected endpoint must name that endpoint, not the one of the earlier session
    // and not nothing ("Could not connect to ubot_ros_bridge at .").
    struct FCase
    {
        const TCHAR* Host;
        int32 Port;
        const TCHAR* Address;
    };
    const FCase Cases[] =
    {
        { TEXT(""), 8268, TEXT("(no host):8268") },
        { TEXT("   "), 8268, TEXT("(no host):8268") },
        { TEXT("127.0.0.1"), 0, TEXT("127.0.0.1:0") },
        { TEXT("127.0.0.1"), -1, TEXT("127.0.0.1:-1") },
    };
    AddExpectedErrorPlain(TEXT("invalid bridge endpoint"), EAutomationExpectedErrorFlags::Contains, static_cast<int32>(UE_ARRAY_COUNT(Cases)));

    for (const FCase& Case : Cases)
    {
        const FString What = FString::Printf(TEXT("Host '%s', port %d"), Case.Host, Case.Port);
        Bridge->ConnectTo(Case.Host, Case.Port);

        TestEqual(*(What + TEXT(": status")), Bridge->GetBridgeStatus(), EUBotROSBridgeStatus::Error);
        TestEqualSensitive(*(What + TEXT(": endpoint")), *Bridge->GetBridgeEndpoint(), Case.Address);
        TestTrue(*(What + TEXT(": last error")), Bridge->GetLastBridgeError().Contains(Case.Address));

        const FUBotRuntimeItem* Item = RuntimeStatus.FindItem(StatusId);
        if (TestNotNull(*(What + TEXT(": item")), Item))
        {
            TestEqualSensitive(*(What + TEXT(": item address")), *Item->Detail, Case.Address);
        }

        const FUBotRuntimeProblem* Problem = RuntimeStatus.FindProblem(StatusId);
        if (TestNotNull(*(What + TEXT(": problem")), Problem))
        {
            TestEqualSensitive(*(What + TEXT(": problem code")), *Problem->Code, TEXT("rosBridgeUnreachable"));
            TestTrue(*(What + TEXT(": address parameter")), Problem->Params.Num() == 1 && Problem->Params[0].Key == TEXT("address") && Problem->Params[0].Value == Case.Address);
            const FString Message = Problem->Message.BuildSourceString();
            TestEqual(*(What + TEXT(": message")), Message, FString::Printf(TEXT("Could not connect to ubot_ros_bridge at %s."), Case.Address));
            TestFalse(*(What + TEXT(": message has no address")), Message.EndsWith(TEXT("at .")));
            TestFalse(*(What + TEXT(": message is not about the earlier endpoint")), Message.Contains(PreviousEndpoint));
        }
    }

    // A valid endpoint replaces the report again.
    TestTrue(TEXT("A valid endpoint connects after the rejected ones"), ConnectBridge(*this, Server, Bridge));
    TestNull(TEXT("Connecting clears the problem"), RuntimeStatus.FindProblem(StatusId));
    TestEqualSensitive(TEXT("The address is the new one again"), *Bridge->GetBridgeEndpoint(), *PreviousEndpoint);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotROSSubsystemSharedStatusTest, "UBotROS.Subsystem.RuntimeStatusPerWorld", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotROSSubsystemSharedStatusTest::RunTest(const FString& Parameters)
{
    using namespace UBotROSSubsystemTests;

    // There is one bridge subsystem per world (PIE with a listen server and clients has several) but one
    // "ros.bridge" item and problem: a world that goes away must not erase what the others report.
    const FString StatusId = TEXT("ros.bridge");
    const FUBotRuntimeStatus& RuntimeStatus = FUBotRuntimeStatus::Get();

    int32 ClosedPort = 0;
    {
        FTestBridgeServer Probe;
        if (!TestTrue(TEXT("Probe listener"), Probe.Start()))
        {
            return false;
        }
        ClosedPort = Probe.GetPort();
    }
    FTestBridgeServer Server;
    if (!TestTrue(TEXT("Test bridge listens"), Server.Start()))
    {
        return false;
    }
    const FString ServerEndpoint = FString::Printf(TEXT("127.0.0.1:%d"), Server.GetPort());

    AddExpectedErrorPlain(TEXT("cannot reach ubot_ros_bridge"), EAutomationExpectedErrorFlags::Contains, 2);
    {
        FScopedBridgeWorld First;
        UUBotROSBridgeSubsystem* FirstBridge = First.Bridge();
        if (!TestNotNull(TEXT("First bridge subsystem"), FirstBridge) || !ConnectBridge(*this, Server, FirstBridge))
        {
            return false;
        }
        TestEqual(TEXT("One world publishes"), UBot::ROSRuntimeStatus::GetPublishingOwnerCount(), 1);

        auto ExpectConnected = [this, &RuntimeStatus, &StatusId, &ServerEndpoint](const TCHAR* When)
        {
            const FUBotRuntimeItem* Item = RuntimeStatus.FindItem(StatusId);
            if (TestNotNull(*FString::Printf(TEXT("%s: the item is still there"), When), Item))
            {
                TestEqualSensitive(*FString::Printf(TEXT("%s: it still says connected"), When), *Item->Value, TEXT("connected"));
                TestEqualSensitive(*FString::Printf(TEXT("%s: it still shows the connected address"), When), *Item->Detail, *ServerEndpoint);
            }
            TestNull(*FString::Printf(TEXT("%s: no problem next to a connected bridge"), When), RuntimeStatus.FindProblem(StatusId));
        };

        // A second world that never connected publishes nothing, and its teardown withdraws nothing.
        {
            FScopedBridgeWorld Idle;
            TestNotNull(TEXT("Idle bridge subsystem"), Idle.Bridge());
        }
        ExpectConnected(TEXT("After an idle world went away"));

        // A second world whose bridge fails: the connected one decides what is shown, before and after.
        {
            FScopedBridgeWorld Failing;
            UUBotROSBridgeSubsystem* FailingBridge = Failing.Bridge();
            if (!TestNotNull(TEXT("Failing bridge subsystem"), FailingBridge))
            {
                return false;
            }
            FailingBridge->ConnectTo(TEXT("127.0.0.1"), ClosedPort);
            TestTrue(TEXT("The second bridge fails"),
                PumpUntil(Server, FailingBridge, [FailingBridge]() { return FailingBridge->GetBridgeStatus() == EUBotROSBridgeStatus::Error; }));
            TestEqual(TEXT("Two worlds publish"), UBot::ROSRuntimeStatus::GetPublishingOwnerCount(), 2);
            ExpectConnected(TEXT("While another world fails"));
        }
        TestEqual(TEXT("Only the first world is left"), UBot::ROSRuntimeStatus::GetPublishingOwnerCount(), 1);
        ExpectConnected(TEXT("After the failing world went away"));

        // Now the first world fails itself; an unrelated world coming and going must not clear that.
        FirstBridge->ConnectTo(TEXT("127.0.0.1"), ClosedPort);
        TestTrue(TEXT("The first bridge fails"), PumpUntil(Server, FirstBridge, [&RuntimeStatus, &StatusId]() { return RuntimeStatus.FindProblem(StatusId) != nullptr; }));
        {
            FScopedBridgeWorld Idle;
            TestNotNull(TEXT("Second idle bridge subsystem"), Idle.Bridge());
        }
        const FUBotRuntimeProblem* Problem = RuntimeStatus.FindProblem(StatusId);
        if (TestNotNull(TEXT("The problem survives an idle world"), Problem))
        {
            TestEqualSensitive(TEXT("It is still the unreachable bridge"), *Problem->Code, TEXT("rosBridgeUnreachable"));
        }
        const FUBotRuntimeItem* Item = RuntimeStatus.FindItem(StatusId);
        TestTrue(TEXT("The failed bridge is still shown"), Item && Item->Value == TEXT("disconnected") && Item->Severity == EUBotRuntimeSeverity::Warning);
    }

    TestEqual(TEXT("No world publishes any more"), UBot::ROSRuntimeStatus::GetPublishingOwnerCount(), 0);
    TestNull(TEXT("The item goes away with the last world"), RuntimeStatus.FindItem(StatusId));
    TestNull(TEXT("No problem outlives the last world"), RuntimeStatus.FindProblem(StatusId));
    return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
