#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "UBotROSProtocol.h"
#include "UBotROSTypes.h"

namespace UBotROSProtocolTests
{
    using namespace UBot::ROSProtocol;

    FString HexString(TConstArrayView<uint8> Bytes)
    {
        FString Result;
        for (int32 Index = 0; Index < Bytes.Num(); ++Index)
        {
            // FString::Printf needs a literal format string, so the separator is a %s argument.
            Result += FString::Printf(TEXT("%s%02x"), Index == 0 ? TEXT("") : TEXT(" "), Bytes[Index]);
        }
        return Result;
    }

    /** Header + payload, with an arbitrary op/version/flags (for malformed input). */
    TArray<uint8> RawFrame(uint8 Op, TConstArrayView<uint8> Payload, uint8 FrameVersion = Version, uint16 Flags = 0)
    {
        TArray<uint8> Frame;
        Frame.SetNumZeroed(HeaderSize + Payload.Num());
        FMemory::Memcpy(Frame.GetData(), Magic, sizeof(Magic));
        Frame[4] = FrameVersion;
        Frame[5] = Op;
        WriteU16LE(Frame.GetData() + 6, Flags);
        WriteU32LE(Frame.GetData() + 8, static_cast<uint32>(Payload.Num()));
        if (Payload.Num() > 0)
        {
            FMemory::Memcpy(Frame.GetData() + HeaderSize, Payload.GetData(), Payload.Num());
        }
        return Frame;
    }

    TArray<uint8> Sequence(int32 Num, uint8 Start)
    {
        TArray<uint8> Bytes;
        Bytes.SetNumUninitialized(Num);
        for (int32 Index = 0; Index < Num; ++Index)
        {
            Bytes[Index] = static_cast<uint8>(Start + Index * 7);
        }
        return Bytes;
    }

    /** Three frames back to back: PUBLISH with a 300-byte body, an empty PING and a JSON SUB_COUNT. */
    TArray<uint8> ThreeFrameStream(TArray<TArray<uint8>>& OutPayloads, TArray<uint8>& OutOps)
    {
        const TArray<uint8> Body = Sequence(300, 3);
        const TArray<uint8> Publish = BuildIdFrame(EOp::Publish, 0xA1B2C3D4u, Body);
        const TArray<uint8> Ping = BuildFrame(EOp::Ping, TConstArrayView<uint8>());
        const TArray<uint8> SubCount = BuildJsonFrame(EOp::SubCount, TEXT("{\"counts\":{\"1\":2}}"));

        OutOps = { static_cast<uint8>(EOp::Publish), static_cast<uint8>(EOp::Ping), static_cast<uint8>(EOp::SubCount) };
        OutPayloads.Reset();
        OutPayloads.Add(TArray<uint8>(TConstArrayView<uint8>(Publish).RightChop(HeaderSize)));
        OutPayloads.Add(TArray<uint8>());
        OutPayloads.Add(TArray<uint8>(TConstArrayView<uint8>(SubCount).RightChop(HeaderSize)));

        TArray<uint8> Stream;
        Stream.Append(Publish);
        Stream.Append(Ping);
        Stream.Append(SubCount);
        return Stream;
    }

    /** Feeds Stream in chunks of the given sizes (the last size repeats) and checks the three frames come out intact. */
    void CheckChunkedParse(FAutomationTestBase& Test, const TCHAR* Label, const TArray<int32>& ChunkSizes)
    {
        TArray<TArray<uint8>> Payloads;
        TArray<uint8> Ops;
        const TArray<uint8> Stream = ThreeFrameStream(Payloads, Ops);

        FFrameParser Parser;
        int32 Offset = 0;
        int32 ChunkIndex = 0;
        bool bFedOk = true;
        while (Offset < Stream.Num())
        {
            const int32 Size = FMath::Min(ChunkSizes[FMath::Min(ChunkIndex, ChunkSizes.Num() - 1)], Stream.Num() - Offset);
            bFedOk &= Parser.Feed(Stream.GetData() + Offset, Size);
            Offset += Size;
            ++ChunkIndex;
        }

        Test.TestTrue(FString::Printf(TEXT("%s: every chunk accepted"), Label), bFedOk);
        Test.TestFalse(FString::Printf(TEXT("%s: no error"), Label), Parser.HasError());
        Test.TestEqual(FString::Printf(TEXT("%s: three frames ready"), Label), Parser.GetReadyFrameCount(), 3);
        Test.TestEqual(FString::Printf(TEXT("%s: nothing partial left"), Label), Parser.GetPartialBytes(), int64(0));
        for (int32 Index = 0; Index < 3; ++Index)
        {
            FWireFrame Frame;
            if (!Test.TestTrue(FString::Printf(TEXT("%s: frame %d pops"), Label, Index), Parser.PopFrame(Frame)))
            {
                return;
            }
            Test.TestEqual(FString::Printf(TEXT("%s: frame %d op"), Label, Index), Frame.Op, Ops[Index]);
            Test.TestEqual(FString::Printf(TEXT("%s: frame %d flags"), Label, Index), Frame.Flags, uint16(0));
            Test.TestTrue(FString::Printf(TEXT("%s: frame %d payload"), Label, Index), Frame.Payload == Payloads[Index]);
        }
        FWireFrame Extra;
        Test.TestFalse(FString::Printf(TEXT("%s: no fourth frame"), Label), Parser.PopFrame(Extra));
    }
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotROSProtocolFrameBuildTest, "UBotROS.Protocol.FrameBuild", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotROSProtocolFrameBuildTest::RunTest(const FString& Parameters)
{
    using namespace UBot::ROSProtocol;
    using namespace UBotROSProtocolTests;

    // Empty PING: header only.
    const TArray<uint8> Ping = BuildFrame(EOp::Ping, TConstArrayView<uint8>());
    TestEqualSensitive(TEXT("PING bytes"), HexString(Ping), FString(TEXT("55 42 52 42 01 0a 00 00 00 00 00 00")));

    // PUBLISH: u32 LE id, then the CDR bytes unchanged; length covers id + body.
    const TArray<uint8> Cdr = { 0x00, 0x01, 0x00, 0x00, 0xAA };
    const TArray<uint8> Publish = BuildIdFrame(EOp::Publish, 0x01020304u, Cdr);
    TestEqualSensitive(TEXT("PUBLISH bytes"), HexString(Publish), FString(TEXT("55 42 52 42 01 05 00 00 09 00 00 00 04 03 02 01 00 01 00 00 aa")));

    // MESSAGE uses the same layout.
    const TArray<uint8> Message = BuildIdFrame(EOp::Message, 7u, TConstArrayView<uint8>());
    TestEqualSensitive(TEXT("MESSAGE with empty body"), HexString(Message), FString(TEXT("55 42 52 42 01 08 00 00 04 00 00 00 07 00 00 00")));

    // Length field is little endian for payloads above 255 bytes.
    const TArray<uint8> Big = BuildFrame(EOp::Publish, Sequence(0x0201, 0));
    TestEqualSensitive(TEXT("Length 0x0201 LE"), HexString(TConstArrayView<uint8>(Big).Left(HeaderSize)), FString(TEXT("55 42 52 42 01 05 00 00 01 02 00 00")));
    TestEqual(TEXT("Frame size"), Big.Num(), HeaderSize + 0x0201);

    // JSON payloads are compact UTF-8 with exactly the contract's field names.
    TestEqualSensitive(TEXT("Client name"), DefaultClientName(), FString(TEXT("uBot UE ")) + ClientVersion);
    TestEqualSensitive(TEXT("HELLO json"), MakeHelloJson(TEXT("uBot UE 0.1.0")), FString(TEXT("{\"protocol\":1,\"client\":\"uBot UE 0.1.0\"}")));
    const TArray<uint8> Hello = BuildJsonFrame(EOp::Hello, MakeHelloJson(TEXT("uBot UE 0.1.0")));
    const FString HelloJson = TEXT("{\"protocol\":1,\"client\":\"uBot UE 0.1.0\"}");
    TestEqual(TEXT("HELLO op"), Hello[5], static_cast<uint8>(EOp::Hello));
    TestEqual(TEXT("HELLO length"), ReadU32LE(Hello.GetData() + 8), static_cast<uint32>(HelloJson.Len()));
    TestEqualSensitive(TEXT("HELLO payload"), FromUtf8(TConstArrayView<uint8>(Hello).RightChop(HeaderSize)), HelloJson);

    const FUBotROSQos SensorQos(EUBotROSReliability::BestEffort, EUBotROSDurability::Volatile, 5);
    TestEqualSensitive(TEXT("ADVERTISE json"), MakeTopicJson(7, TEXT("/ubot/imu"), TEXT("sensor_msgs/msg/Imu"), SensorQos),
        FString(TEXT("{\"id\":7,\"topic\":\"/ubot/imu\",\"type\":\"sensor_msgs/msg/Imu\",\"qos\":{\"reliability\":\"best_effort\",\"durability\":\"volatile\",\"depth\":5}}")));
    const FUBotROSQos LatchedQos(EUBotROSReliability::Reliable, EUBotROSDurability::TransientLocal, 1);
    TestEqualSensitive(TEXT("QoS json latched"), MakeQosJson(LatchedQos), FString(TEXT("{\"reliability\":\"reliable\",\"durability\":\"transient_local\",\"depth\":1}")));
    const FUBotROSQos ZeroDepth(EUBotROSReliability::Reliable, EUBotROSDurability::Volatile, 0);
    TestEqualSensitive(TEXT("Depth is clamped to 1"), MakeQosJson(ZeroDepth), FString(TEXT("{\"reliability\":\"reliable\",\"durability\":\"volatile\",\"depth\":1}")));
    TestEqualSensitive(TEXT("UNADVERTISE json"), MakeIdJson(4294967295u), FString(TEXT("{\"id\":4294967295}")));
    TestEqualSensitive(TEXT("SUB_COUNT json"), MakeSubCountJson({ TPair<uint32, int32>(3, 0), TPair<uint32, int32>(12, 2) }), FString(TEXT("{\"counts\":{\"3\":0,\"12\":2}}")));
    TestEqualSensitive(TEXT("ERROR json without id"), MakeErrorJson(TOptional<uint32>(), TEXT("unknown_op"), TEXT("op 99")),
        FString(TEXT("{\"id\":null,\"code\":\"unknown_op\",\"message\":\"op 99\"}")));

    // Parsers accept what the builders produce (the test bridge relies on them).
    {
        FTopicRequest Request;
        const FUBotROSQos StateQos(EUBotROSReliability::Reliable, EUBotROSDurability::Volatile, 10);
        if (TestTrue(TEXT("ADVERTISE parses"), ParseTopicRequest(ToUtf8(MakeTopicJson(42, TEXT("/a/b"), TEXT("nav_msgs/msg/Odometry"), StateQos)), Request)))
        {
            TestEqual(TEXT("Parsed id"), Request.Id, 42u);
            TestEqualSensitive(TEXT("Parsed topic"), Request.Topic, FString(TEXT("/a/b")));
            TestEqualSensitive(TEXT("Parsed type"), Request.Type, FString(TEXT("nav_msgs/msg/Odometry")));
            TestTrue(TEXT("Parsed QoS"), Request.Qos == StateQos);
        }
        TestFalse(TEXT("Unknown reliability fails"), ParseTopicRequest(ToUtf8(TEXT("{\"id\":1,\"topic\":\"/a\",\"type\":\"a/msg/B\",\"qos\":{\"reliability\":\"sometimes\",\"durability\":\"volatile\",\"depth\":1}}")), Request));
        TestFalse(TEXT("Missing qos fails"), ParseTopicRequest(ToUtf8(TEXT("{\"id\":1,\"topic\":\"/a\",\"type\":\"a/msg/B\"}")), Request));
        TestFalse(TEXT("Negative id fails"), ParseTopicRequest(ToUtf8(TEXT("{\"id\":-1,\"topic\":\"/a\",\"type\":\"a/msg/B\",\"qos\":{\"reliability\":\"reliable\",\"durability\":\"volatile\",\"depth\":1}}")), Request));

        uint32 Id = 0;
        TestTrue(TEXT("UNSUBSCRIBE parses"), ParseIdRequest(ToUtf8(MakeIdJson(9)), Id) && Id == 9u);
        TestFalse(TEXT("Fractional id fails"), ParseIdRequest(ToUtf8(TEXT("{\"id\":1.5}")), Id));

        FHelloAck Ack;
        if (TestTrue(TEXT("HELLO_ACK parses"), ParseHelloAck(ToUtf8(MakeHelloAckJson(TEXT("ubot_ros_bridge 0.1.0"), TEXT("humble"))), Ack)))
        {
            TestEqual(TEXT("HELLO_ACK protocol"), Ack.Protocol, 1);
            TestEqualSensitive(TEXT("HELLO_ACK server"), Ack.Server, FString(TEXT("ubot_ros_bridge 0.1.0")));
            TestEqualSensitive(TEXT("HELLO_ACK distro"), Ack.RosDistro, FString(TEXT("humble")));
        }
        TestFalse(TEXT("HELLO_ACK without protocol fails"), ParseHelloAck(ToUtf8(TEXT("{\"server\":\"x\"}")), Ack));
        TestFalse(TEXT("Non-JSON fails"), ParseHelloAck(ToUtf8(TEXT("hello")), Ack));

        FHello HelloInfo;
        TestTrue(TEXT("HELLO parses"), ParseHello(ToUtf8(MakeHelloJson(TEXT("uBot UE 0.1.0"))), HelloInfo) && HelloInfo.Protocol == 1 && HelloInfo.Client.Equals(TEXT("uBot UE 0.1.0"), ESearchCase::CaseSensitive));

        TArray<TPair<uint32, int32>> Counts;
        if (TestTrue(TEXT("SUB_COUNT parses"), ParseSubCount(ToUtf8(TEXT("{\"counts\":{\"1\":2,\"x\":3,\"4\":-1,\"5\":1.5,\"6\":7,\"-2\":1,\"4294967296\":1}}")), Counts)))
        {
            TestEqual(TEXT("Invalid entries are skipped"), Counts.Num(), 2);
            if (Counts.Num() == 2)
            {
                TestTrue(TEXT("Entry 1"), Counts[0].Key == 1u && Counts[0].Value == 2);
                TestTrue(TEXT("Entry 6"), Counts[1].Key == 6u && Counts[1].Value == 7);
            }
        }
        TestFalse(TEXT("SUB_COUNT without counts fails"), ParseSubCount(ToUtf8(TEXT("{\"n\":1}")), Counts));

        FErrorInfo Error;
        if (TestTrue(TEXT("ERROR with id parses"), ParseError(ToUtf8(MakeErrorJson(TOptional<uint32>(5u), TEXT("type_conflict"), TEXT("x"))), Error)))
        {
            TestTrue(TEXT("ERROR id"), Error.Id.IsSet() && Error.Id.GetValue() == 5u);
            TestEqualSensitive(TEXT("ERROR code"), Error.Code, FString(TEXT("type_conflict")));
            TestEqualSensitive(TEXT("ERROR message"), Error.Message, FString(TEXT("x")));
        }
        TestTrue(TEXT("ERROR with null id parses"), ParseError(ToUtf8(MakeErrorJson(TOptional<uint32>(), TEXT("c"), TEXT("m"))), Error) && !Error.Id.IsSet());

        uint32 SplitId = 0;
        TConstArrayView<uint8> SplitBody;
        TestTrue(TEXT("Split PUBLISH payload"), SplitIdPayload(TConstArrayView<uint8>(Publish).RightChop(HeaderSize), SplitId, SplitBody) && SplitId == 0x01020304u && TArray<uint8>(SplitBody) == Cdr);
        const TArray<uint8> Short = { 1, 2, 3 };
        TestFalse(TEXT("Split rejects < 4 bytes"), SplitIdPayload(Short, SplitId, SplitBody));
    }

    // Non-ASCII text survives the UTF-8 round trip and the byte length (not the character count) is framed.
    {
        FString Text = TEXT("caf");
        Text.AppendChar(static_cast<TCHAR>(0x00E9));
        Text.AppendChar(TEXT(' '));
        Text.AppendChar(static_cast<TCHAR>(0x4E2D));
        const TArray<uint8> Utf8 = ToUtf8(Text);
        TestEqual(TEXT("UTF-8 byte count"), Utf8.Num(), 3 + 2 + 1 + 3);
        TestEqualSensitive(TEXT("UTF-8 round trip"), FromUtf8(Utf8), Text);
        const TArray<uint8> Frame = BuildJsonFrame(EOp::Error, Text);
        TestEqual(TEXT("Frame length counts bytes"), ReadU32LE(Frame.GetData() + 8), static_cast<uint32>(Utf8.Num()));
    }

    // Ops.
    TestTrue(TEXT("Op 1 known"), IsKnownOp(1));
    TestTrue(TEXT("Op 12 known"), IsKnownOp(12));
    TestFalse(TEXT("Op 0 unknown"), IsKnownOp(0));
    TestFalse(TEXT("Op 13 unknown"), IsKnownOp(13));
    TestEqualSensitive(TEXT("Op name"), OpToString(9), FString(TEXT("SUB_COUNT")));
    TestEqualSensitive(TEXT("Unknown op name"), OpToString(200), FString(TEXT("UNKNOWN(200)")));
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotROSProtocolParserTest, "UBotROS.Protocol.IncrementalParser", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotROSProtocolParserTest::RunTest(const FString& Parameters)
{
    using namespace UBot::ROSProtocol;
    using namespace UBotROSProtocolTests;

    // Everything in one read (several frames per read).
    CheckChunkedParse(*this, TEXT("Single read"), { 100000 });
    // One byte at a time: headers and payloads split everywhere.
    CheckChunkedParse(*this, TEXT("Byte by byte"), { 1 });
    // Split inside the first header, then inside the payload, then reads spanning frame boundaries.
    CheckChunkedParse(*this, TEXT("Irregular reads"), { 5, 9, 150, 3, 200, 13, 7 });
    // Header exactly, payload exactly.
    CheckChunkedParse(*this, TEXT("Header-sized reads"), { HeaderSize, 304, HeaderSize, HeaderSize, 1000 });

    // Every split point of the stream into two reads.
    {
        TArray<TArray<uint8>> Payloads;
        TArray<uint8> Ops;
        const TArray<uint8> Stream = ThreeFrameStream(Payloads, Ops);
        int32 Failures = 0;
        for (int32 Split = 0; Split <= Stream.Num(); ++Split)
        {
            FFrameParser Parser;
            Parser.Feed(Stream.GetData(), Split);
            Parser.Feed(Stream.GetData() + Split, Stream.Num() - Split);
            bool bOk = !Parser.HasError() && Parser.GetReadyFrameCount() == 3;
            for (int32 Index = 0; bOk && Index < 3; ++Index)
            {
                FWireFrame Frame;
                bOk = Parser.PopFrame(Frame) && Frame.Op == Ops[Index] && Frame.Payload == Payloads[Index];
            }
            Failures += bOk ? 0 : 1;
        }
        TestEqual(TEXT("Every two-read split yields the same frames"), Failures, 0);
    }

    // Frames become available as soon as they are complete, before later frames arrive.
    {
        FFrameParser Parser;
        const TArray<uint8> First = BuildJsonFrame(EOp::HelloAck, MakeHelloAckJson(TEXT("s"), TEXT("humble")));
        const TArray<uint8> Second = BuildIdFrame(EOp::Message, 3, Sequence(40, 1));
        TArray<uint8> Stream = First;
        Stream.Append(TConstArrayView<uint8>(Second).Left(20));
        TestTrue(TEXT("Partial feed accepted"), Parser.Feed(Stream));
        TestEqual(TEXT("First frame ready while the second is partial"), Parser.GetReadyFrameCount(), 1);
        TestEqual(TEXT("Partial bytes of the second frame"), Parser.GetPartialBytes(), int64(20));
        FWireFrame Frame;
        TestTrue(TEXT("HELLO_ACK pops"), Parser.PopFrame(Frame) && Frame.Op == static_cast<uint8>(EOp::HelloAck));
        TestFalse(TEXT("Nothing else yet"), Parser.PopFrame(Frame));
        TestTrue(TEXT("Rest accepted"), Parser.Feed(TConstArrayView<uint8>(Second).RightChop(20)));
        TestTrue(TEXT("MESSAGE pops"), Parser.PopFrame(Frame) && Frame.Op == static_cast<uint8>(EOp::Message) && Frame.Payload.Num() == 44);
    }

    // Unknown ops and non-zero flags are passed through (flags are reserved and ignored, unknown ops answered by the receiver).
    {
        FFrameParser Parser;
        const TArray<uint8> Payload = { 9, 8, 7 };
        TestTrue(TEXT("Unknown op accepted"), Parser.Feed(RawFrame(200, Payload, Version, 0xBEEF)));
        FWireFrame Frame;
        if (TestTrue(TEXT("Unknown op frame pops"), Parser.PopFrame(Frame)))
        {
            TestEqual(TEXT("Raw op kept"), Frame.Op, uint8(200));
            TestEqual(TEXT("Flags kept"), Frame.Flags, uint16(0xBEEF));
            TestTrue(TEXT("Payload kept"), Frame.Payload == Payload);
        }
    }

    // A payload exactly at the limit is fine.
    {
        FFrameParser Parser(16);
        TestTrue(TEXT("Payload at the limit accepted"), Parser.Feed(BuildFrame(EOp::Publish, Sequence(16, 0))) && Parser.GetReadyFrameCount() == 1);
    }
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotROSProtocolRejectionTest, "UBotROS.Protocol.Rejection", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotROSProtocolRejectionTest::RunTest(const FString& Parameters)
{
    using namespace UBot::ROSProtocol;
    using namespace UBotROSProtocolTests;

    // Bad magic: rejected once the header is complete, and the error is sticky.
    {
        FFrameParser Parser;
        TArray<uint8> Frame = BuildFrame(EOp::Ping, TConstArrayView<uint8>());
        Frame[1] = 'X';
        TestTrue(TEXT("Partial header is not judged yet"), Parser.Feed(TConstArrayView<uint8>(Frame).Left(HeaderSize - 1)));
        TestFalse(TEXT("Bad magic rejected"), Parser.Feed(TConstArrayView<uint8>(Frame).RightChop(HeaderSize - 1)));
        TestEqual(TEXT("Bad magic error"), Parser.GetError(), EParseError::BadMagic);
        TestFalse(TEXT("Error is sticky"), Parser.Feed(BuildFrame(EOp::Ping, TConstArrayView<uint8>())));
        TestEqual(TEXT("No frames after the error"), Parser.GetReadyFrameCount(), 0);
        Parser.Reset();
        TestTrue(TEXT("Reset clears the error"), !Parser.HasError() && Parser.Feed(BuildFrame(EOp::Ping, TConstArrayView<uint8>())) && Parser.GetReadyFrameCount() == 1);
    }

    // Bad magic after a valid frame: the valid frame stays available.
    {
        FFrameParser Parser;
        TArray<uint8> Stream = BuildFrame(EOp::Pong, TConstArrayView<uint8>());
        const TArray<uint8> Garbage = { 'G', 'E', 'T', ' ', '/', ' ', 'H', 'T', 'T', 'P', '/', '1' };
        Stream.Append(Garbage);
        TestFalse(TEXT("HTTP garbage rejected"), Parser.Feed(Stream));
        TestEqual(TEXT("Garbage error"), Parser.GetError(), EParseError::BadMagic);
        FWireFrame Frame;
        TestTrue(TEXT("Frame before the garbage still pops"), Parser.PopFrame(Frame) && Frame.Op == static_cast<uint8>(EOp::Pong));
    }

    // Unsupported version.
    {
        FFrameParser Parser;
        TestFalse(TEXT("Version 2 rejected"), Parser.Feed(RawFrame(static_cast<uint8>(EOp::Ping), TConstArrayView<uint8>(), 2)));
        TestEqual(TEXT("Version error"), Parser.GetError(), EParseError::BadVersion);
        FFrameParser ZeroVersion;
        TestFalse(TEXT("Version 0 rejected"), ZeroVersion.Feed(RawFrame(static_cast<uint8>(EOp::Ping), TConstArrayView<uint8>(), 0)));
        TestEqual(TEXT("Version 0 error"), ZeroVersion.GetError(), EParseError::BadVersion);
    }

    // Oversize: detected from the header alone, before the payload arrives.
    {
        FFrameParser Parser(16);
        uint8 Header[HeaderSize];
        WriteHeader(Header, EOp::Publish, 17);
        TestFalse(TEXT("Length above the limit rejected"), Parser.Feed(Header, HeaderSize));
        TestEqual(TEXT("Oversize error"), Parser.GetError(), EParseError::Oversize);
        TestEqual(TEXT("Nothing buffered"), Parser.GetReadyFrameCount(), 0);

        FFrameParser DefaultLimit;
        WriteHeader(Header, EOp::Message, DefaultMaxPayloadBytes + 1);
        TestFalse(TEXT("Default limit is 64 MiB"), DefaultLimit.Feed(Header, HeaderSize));
        TestEqual(TEXT("Default limit oversize error"), DefaultLimit.GetError(), EParseError::Oversize);

        FFrameParser Huge;
        WriteHeader(Header, EOp::Message, 0xFFFFFFFFu);
        Huge.SetMaxPayloadBytes(0xFFFFFFFFu);
        TestFalse(TEXT("Lengths beyond int32 are rejected even with a huge limit"), Huge.Feed(Header, HeaderSize));
    }

    TestEqualSensitive(TEXT("Error text"), FString(ParseErrorToString(EParseError::Oversize)), FString(TEXT("payload exceeds the size limit")));
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotROSProtocolNamesTest, "UBotROS.Protocol.Names", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotROSProtocolNamesTest::RunTest(const FString& Parameters)
{
    using namespace UBot::ROSProtocol;

    TestTrue(TEXT("/clock"), IsValidTopicName(TEXT("/clock")));
    TestTrue(TEXT("/tf_static"), IsValidTopicName(TEXT("/tf_static")));
    TestTrue(TEXT("/ubot_e2e/imu"), IsValidTopicName(TEXT("/ubot_e2e/imu")));
    TestTrue(TEXT("/robot1/camera/color/image_raw"), IsValidTopicName(TEXT("/robot1/camera/color/image_raw")));
    TestTrue(TEXT("/_private/x9"), IsValidTopicName(TEXT("/_private/x9")));
    TestFalse(TEXT("Relative name"), IsValidTopicName(TEXT("clock")));
    TestFalse(TEXT("Root only"), IsValidTopicName(TEXT("/")));
    TestFalse(TEXT("Empty"), IsValidTopicName(TEXT("")));
    TestFalse(TEXT("Trailing slash"), IsValidTopicName(TEXT("/a/")));
    TestFalse(TEXT("Double slash"), IsValidTopicName(TEXT("/a//b")));
    TestFalse(TEXT("Token starts with a digit"), IsValidTopicName(TEXT("/a/9b")));
    TestFalse(TEXT("Hyphen"), IsValidTopicName(TEXT("/a-b")));
    TestFalse(TEXT("Space"), IsValidTopicName(TEXT("/a b")));
    TestFalse(TEXT("Private namespace substitution"), IsValidTopicName(TEXT("~/a")));
    TestFalse(TEXT("Too long"), IsValidTopicName(TEXT("/") + FString::ChrN(255, TEXT('a'))));

    TestTrue(TEXT("sensor_msgs/msg/Imu"), IsValidMessageType(TEXT("sensor_msgs/msg/Imu")));
    TestTrue(TEXT("tf2_msgs/msg/TFMessage"), IsValidMessageType(TEXT("tf2_msgs/msg/TFMessage")));
    TestFalse(TEXT("Short form"), IsValidMessageType(TEXT("sensor_msgs/Imu")));
    TestFalse(TEXT("Service"), IsValidMessageType(TEXT("std_srvs/srv/Empty")));
    TestFalse(TEXT("Empty package"), IsValidMessageType(TEXT("/msg/Imu")));
    TestFalse(TEXT("Trailing slash"), IsValidMessageType(TEXT("sensor_msgs/msg/")));
    TestFalse(TEXT("Extra part"), IsValidMessageType(TEXT("a/msg/B/C")));
    TestFalse(TEXT("Bad character"), IsValidMessageType(TEXT("sensor-msgs/msg/Imu")));
    return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
