#pragma once

#include "CoreMinimal.h"
#include "Containers/ArrayView.h"
#include "Containers/Deque.h"
#include "UBotROSTypes.h"

/**
 * Wire protocol v1 between UBotROS (TCP client) and ubot_ros_bridge (TCP server).
 * See docs/PROTOCOL.md. Pure code: no sockets, no threads, no UObjects, so everything here is
 * unit tested in isolation (UBotROS.Protocol.*).
 *
 * Frame = 12-byte header + payload:
 *   0  4  magic "UBRB" (55 42 52 42)
 *   4  1  version (1)
 *   5  1  op
 *   6  2  flags (u16 LE, reserved: sent as 0, ignored on receive)
 *   8  4  payload length N (u32 LE)
 *  12  N  payload
 */
namespace UBot::ROSProtocol
{
    inline constexpr uint8 Magic[4] = { 0x55, 0x42, 0x52, 0x42 };
    inline constexpr uint8 Version = 1;
    /** Value of the "protocol" field in HELLO / HELLO_ACK. */
    inline constexpr int32 ProtocolVersion = 1;
    inline constexpr int32 HeaderSize = 12;
    /** Size of the u32 id that prefixes PUBLISH and MESSAGE payloads. */
    inline constexpr int32 IdPrefixSize = 4;
    inline constexpr uint32 DefaultMaxPayloadBytes = 64u * 1024u * 1024u;
    inline constexpr uint16 DefaultPort = 8268;
    /** The client sends PING this often while connected. */
    inline constexpr double PingIntervalSeconds = 2.0;
    /** A peer that has not received any frame for this long drops the connection. */
    inline constexpr double IdleTimeoutSeconds = 10.0;
    /** Version reported in HELLO ("uBot UE <version>"); keep in sync with UBotROS.uplugin VersionName. */
    inline constexpr const TCHAR* ClientVersion = TEXT("0.1.0");

    enum class EOp : uint8
    {
        Hello = 1,
        HelloAck = 2,
        Advertise = 3,
        Unadvertise = 4,
        Publish = 5,
        Subscribe = 6,
        Unsubscribe = 7,
        Message = 8,
        SubCount = 9,
        Ping = 10,
        Pong = 11,
        Error = 12
    };

    /** True for the ops 1..12 defined by protocol v1. */
    bool IsKnownOp(uint8 Op);

    /** Upper-case protocol name of an op ("PUBLISH"), or "UNKNOWN(<n>)". */
    FString OpToString(uint8 Op);

    // ---------------------------------------------------------------------------------------------
    // Little-endian helpers
    // ---------------------------------------------------------------------------------------------

    inline void WriteU16LE(uint8* Out, uint16 Value)
    {
        Out[0] = static_cast<uint8>(Value & 0xFF);
        Out[1] = static_cast<uint8>((Value >> 8) & 0xFF);
    }

    inline void WriteU32LE(uint8* Out, uint32 Value)
    {
        Out[0] = static_cast<uint8>(Value & 0xFF);
        Out[1] = static_cast<uint8>((Value >> 8) & 0xFF);
        Out[2] = static_cast<uint8>((Value >> 16) & 0xFF);
        Out[3] = static_cast<uint8>((Value >> 24) & 0xFF);
    }

    inline uint16 ReadU16LE(const uint8* In)
    {
        return static_cast<uint16>(In[0] | (In[1] << 8));
    }

    inline uint32 ReadU32LE(const uint8* In)
    {
        return static_cast<uint32>(In[0]) | (static_cast<uint32>(In[1]) << 8) | (static_cast<uint32>(In[2]) << 16) | (static_cast<uint32>(In[3]) << 24);
    }

    // ---------------------------------------------------------------------------------------------
    // Frame building
    // ---------------------------------------------------------------------------------------------

    /** Writes the 12-byte header (flags 0) to Out, which must hold HeaderSize bytes. */
    void WriteHeader(uint8* Out, EOp Op, uint32 PayloadLength);

    /** Header + Payload. */
    TArray<uint8> BuildFrame(EOp Op, TConstArrayView<uint8> Payload);

    /** Header + u32 LE Id + Body (PUBLISH, MESSAGE). */
    TArray<uint8> BuildIdFrame(EOp Op, uint32 Id, TConstArrayView<uint8> Body);

    /** Header + UTF-8 encoded Json (no terminator). */
    TArray<uint8> BuildJsonFrame(EOp Op, const FString& Json);

    /** UTF-8 bytes of Text, without terminator. */
    TArray<uint8> ToUtf8(const FString& Text);

    /** Decodes a UTF-8 payload (no terminator required). */
    FString FromUtf8(TConstArrayView<uint8> Utf8);

    // ---------------------------------------------------------------------------------------------
    // JSON payloads (compact, UTF-8 on the wire)
    // ---------------------------------------------------------------------------------------------

    /** "uBot UE <ClientVersion>". */
    FString DefaultClientName();

    /** {"protocol":1,"client":"<ClientName>"} */
    FString MakeHelloJson(const FString& ClientName);

    /** {"protocol":<Protocol>,"server":"<ServerName>","ros_distro":"<RosDistro>"} (used by test servers). */
    FString MakeHelloAckJson(const FString& ServerName, const FString& RosDistro, int32 Protocol = ProtocolVersion);

    /** {"reliability":"reliable"|"best_effort","durability":"volatile"|"transient_local","depth":n}; depth is clamped to >= 1. */
    FString MakeQosJson(const FUBotROSQos& Qos);

    /** ADVERTISE / SUBSCRIBE: {"id":Id,"topic":Topic,"type":Type,"qos":{...}} */
    FString MakeTopicJson(uint32 Id, const FString& Topic, const FString& Type, const FUBotROSQos& Qos);

    /** UNADVERTISE / UNSUBSCRIBE: {"id":Id} */
    FString MakeIdJson(uint32 Id);

    /** SUB_COUNT: {"counts":{"<id>":n,...}} (used by test servers). */
    FString MakeSubCountJson(const TArray<TPair<uint32, int32>>& Counts);

    /** ERROR: {"id":Id or null,"code":Code,"message":Message} (used by test servers). */
    FString MakeErrorJson(TOptional<uint32> Id, const FString& Code, const FString& Message);

    struct FHelloAck
    {
        int32 Protocol = 0;
        FString Server;
        FString RosDistro;
    };

    /** HELLO_ACK. Requires an integer "protocol"; "server" and "ros_distro" are optional strings. */
    bool ParseHelloAck(TConstArrayView<uint8> Payload, FHelloAck& Out);

    struct FHello
    {
        int32 Protocol = 0;
        FString Client;
    };

    /** HELLO (used by test servers). */
    bool ParseHello(TConstArrayView<uint8> Payload, FHello& Out);

    struct FTopicRequest
    {
        uint32 Id = 0;
        FString Topic;
        FString Type;
        FUBotROSQos Qos;
    };

    /** ADVERTISE / SUBSCRIBE (used by test servers). Unknown QoS strings fail the parse. */
    bool ParseTopicRequest(TConstArrayView<uint8> Payload, FTopicRequest& Out);

    /** UNADVERTISE / UNSUBSCRIBE (used by test servers). */
    bool ParseIdRequest(TConstArrayView<uint8> Payload, uint32& OutId);

    /** SUB_COUNT. Entries with a non-numeric id or a negative/non-integer count are skipped. */
    bool ParseSubCount(TConstArrayView<uint8> Payload, TArray<TPair<uint32, int32>>& OutCounts);

    struct FErrorInfo
    {
        TOptional<uint32> Id;
        FString Code;
        FString Message;
    };

    /** ERROR. "id" may be null or absent. */
    bool ParseError(TConstArrayView<uint8> Payload, FErrorInfo& Out);

    /** Splits a PUBLISH / MESSAGE payload into its u32 LE id and the CDR body. False if shorter than 4 bytes. */
    bool SplitIdPayload(TConstArrayView<uint8> Payload, uint32& OutId, TConstArrayView<uint8>& OutBody);

    // ---------------------------------------------------------------------------------------------
    // Names
    // ---------------------------------------------------------------------------------------------

    /**
     * Fully qualified ROS 2 topic name: starts with '/', tokens of [A-Za-z0-9_] separated by single
     * '/', no token starts with a digit, no trailing '/', at most 255 characters.
     */
    bool IsValidTopicName(const FString& Topic);

    /** "<package>/msg/<Message>" with identifier-like package and message names. */
    bool IsValidMessageType(const FString& Type);

    // ---------------------------------------------------------------------------------------------
    // Incremental frame parser
    // ---------------------------------------------------------------------------------------------

    struct FWireFrame
    {
        /** Raw op byte; may be an op this side does not know. */
        uint8 Op = 0;
        uint16 Flags = 0;
        TArray<uint8> Payload;
    };

    enum class EParseError : uint8
    {
        None,
        BadMagic,
        BadVersion,
        Oversize
    };

    const TCHAR* ParseErrorToString(EParseError Error);

    /**
     * Reassembles frames from arbitrary stream chunks (split headers, split payloads, several frames
     * per chunk). A malformed header (bad magic, bad version, payload length above the limit) puts the
     * parser in a sticky error state: the connection must be closed. The header is validated as soon
     * as its 12 bytes are available, before any payload memory is allocated.
     */
    class FFrameParser
    {
    public:
        explicit FFrameParser(uint32 InMaxPayloadBytes = DefaultMaxPayloadBytes);

        /** Forgets buffered bytes, ready frames and the error state. */
        void Reset();

        void SetMaxPayloadBytes(uint32 InMaxPayloadBytes);
        uint32 GetMaxPayloadBytes() const { return MaxPayloadBytes; }

        /** Consumes Num bytes. Returns false if the stream is (or already was) malformed. */
        bool Feed(const uint8* Data, int32 Num);

        bool Feed(TConstArrayView<uint8> Data)
        {
            return Feed(Data.GetData(), Data.Num());
        }

        /** Moves the oldest complete frame to OutFrame. */
        bool PopFrame(FWireFrame& OutFrame);

        int32 GetReadyFrameCount() const { return Ready.Num(); }
        bool HasError() const { return Error != EParseError::None; }
        EParseError GetError() const { return Error; }

        /** Bytes of the incomplete frame currently being assembled (header and payload). */
        int64 GetPartialBytes() const;

    private:
        void CompleteCurrentFrame();

        uint32 MaxPayloadBytes = DefaultMaxPayloadBytes;
        uint8 HeaderBytes[HeaderSize] = {};
        int32 HeaderFill = 0;
        bool bHaveHeader = false;
        uint32 PayloadLength = 0;
        uint32 PayloadFill = 0;
        FWireFrame Current;
        TDeque<FWireFrame> Ready;
        EParseError Error = EParseError::None;
    };
}
