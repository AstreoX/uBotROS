#include "UBotROSProtocol.h"

#include "Containers/StringConv.h"
#include "Dom/JsonObject.h"
#include "Dom/JsonValue.h"
#include "Policies/CondensedJsonPrintPolicy.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"
#include "Serialization/JsonWriter.h"

namespace UBot::ROSProtocol
{
    namespace Private
    {
        using FCompactJsonWriter = TJsonWriter<TCHAR, TCondensedJsonPrintPolicy<TCHAR>>;
        using FCompactJsonWriterFactory = TJsonWriterFactory<TCHAR, TCondensedJsonPrintPolicy<TCHAR>>;

        constexpr int32 MaxTopicLength = 255;

        const TCHAR* ReliabilityToString(EUBotROSReliability Reliability)
        {
            return Reliability == EUBotROSReliability::Reliable ? TEXT("reliable") : TEXT("best_effort");
        }

        const TCHAR* DurabilityToString(EUBotROSDurability Durability)
        {
            return Durability == EUBotROSDurability::TransientLocal ? TEXT("transient_local") : TEXT("volatile");
        }

        void WriteQos(FCompactJsonWriter& Writer, const FUBotROSQos& Qos)
        {
            Writer.WriteObjectStart(TEXT("qos"));
            Writer.WriteValue(TEXT("reliability"), ReliabilityToString(Qos.Reliability));
            Writer.WriteValue(TEXT("durability"), DurabilityToString(Qos.Durability));
            Writer.WriteValue(TEXT("depth"), FMath::Max(1, static_cast<int32>(Qos.Depth)));
            Writer.WriteObjectEnd();
        }

        TSharedPtr<FJsonObject> ParseObject(TConstArrayView<uint8> Payload)
        {
            const FString Text = FromUtf8(Payload);
            TSharedPtr<FJsonObject> Object;
            const TSharedRef<TJsonReader<TCHAR>> Reader = TJsonReaderFactory<TCHAR>::Create(Text);
            if (!FJsonSerializer::Deserialize(Reader, Object) || !Object.IsValid())
            {
                return nullptr;
            }
            return Object;
        }

        /** Integral JSON number within [Min, Max]. */
        bool GetIntegerField(const FJsonObject& Object, const TCHAR* Field, int64 Min, int64 Max, int64& OutValue)
        {
            const TSharedPtr<FJsonValue> Value = Object.TryGetField(Field);
            if (!Value.IsValid() || Value->Type != EJson::Number)
            {
                return false;
            }
            const double Number = Value->AsNumber();
            if (!FMath::IsFinite(Number) || FMath::RoundToDouble(Number) != Number)
            {
                return false;
            }
            if (Number < static_cast<double>(Min) || Number > static_cast<double>(Max))
            {
                return false;
            }
            OutValue = static_cast<int64>(Number);
            return true;
        }

        bool GetId(const FJsonObject& Object, uint32& OutId)
        {
            int64 Value = 0;
            if (!GetIntegerField(Object, TEXT("id"), 0, MAX_uint32, Value))
            {
                return false;
            }
            OutId = static_cast<uint32>(Value);
            return true;
        }

        /** Plain decimal digits ("0" .. "4294967295"), as used for the keys of SUB_COUNT "counts". */
        bool ParseDecimalId(const FString& Text, uint32& OutId)
        {
            if (Text.IsEmpty() || Text.Len() > 10)
            {
                return false;
            }
            uint64 Value = 0;
            for (const TCHAR Char : Text)
            {
                if (Char < TEXT('0') || Char > TEXT('9'))
                {
                    return false;
                }
                Value = Value * 10 + static_cast<uint64>(Char - TEXT('0'));
            }
            if (Value > MAX_uint32)
            {
                return false;
            }
            OutId = static_cast<uint32>(Value);
            return true;
        }

        bool IsIdentifierStart(TCHAR Char)
        {
            return (Char >= TEXT('a') && Char <= TEXT('z')) || (Char >= TEXT('A') && Char <= TEXT('Z')) || Char == TEXT('_');
        }

        bool IsIdentifierChar(TCHAR Char)
        {
            return IsIdentifierStart(Char) || (Char >= TEXT('0') && Char <= TEXT('9'));
        }

        bool IsIdentifier(const FString& Token)
        {
            if (Token.IsEmpty() || !IsIdentifierStart(Token[0]))
            {
                return false;
            }
            for (const TCHAR Char : Token)
            {
                if (!IsIdentifierChar(Char))
                {
                    return false;
                }
            }
            return true;
        }
    }

    bool IsKnownOp(uint8 Op)
    {
        return Op >= static_cast<uint8>(EOp::Hello) && Op <= static_cast<uint8>(EOp::Error);
    }

    FString OpToString(uint8 Op)
    {
        switch (static_cast<EOp>(Op))
        {
        case EOp::Hello:
            return TEXT("HELLO");
        case EOp::HelloAck:
            return TEXT("HELLO_ACK");
        case EOp::Advertise:
            return TEXT("ADVERTISE");
        case EOp::Unadvertise:
            return TEXT("UNADVERTISE");
        case EOp::Publish:
            return TEXT("PUBLISH");
        case EOp::Subscribe:
            return TEXT("SUBSCRIBE");
        case EOp::Unsubscribe:
            return TEXT("UNSUBSCRIBE");
        case EOp::Message:
            return TEXT("MESSAGE");
        case EOp::SubCount:
            return TEXT("SUB_COUNT");
        case EOp::Ping:
            return TEXT("PING");
        case EOp::Pong:
            return TEXT("PONG");
        case EOp::Error:
            return TEXT("ERROR");
        default:
            return FString::Printf(TEXT("UNKNOWN(%u)"), static_cast<uint32>(Op));
        }
    }

    void WriteHeader(uint8* Out, EOp Op, uint32 PayloadLength)
    {
        Out[0] = Magic[0];
        Out[1] = Magic[1];
        Out[2] = Magic[2];
        Out[3] = Magic[3];
        Out[4] = Version;
        Out[5] = static_cast<uint8>(Op);
        WriteU16LE(Out + 6, 0);
        WriteU32LE(Out + 8, PayloadLength);
    }

    TArray<uint8> BuildFrame(EOp Op, TConstArrayView<uint8> Payload)
    {
        TArray<uint8> Frame;
        Frame.SetNumUninitialized(HeaderSize + Payload.Num());
        WriteHeader(Frame.GetData(), Op, static_cast<uint32>(Payload.Num()));
        if (Payload.Num() > 0)
        {
            FMemory::Memcpy(Frame.GetData() + HeaderSize, Payload.GetData(), Payload.Num());
        }
        return Frame;
    }

    TArray<uint8> BuildIdFrame(EOp Op, uint32 Id, TConstArrayView<uint8> Body)
    {
        TArray<uint8> Frame;
        Frame.SetNumUninitialized(HeaderSize + IdPrefixSize + Body.Num());
        WriteHeader(Frame.GetData(), Op, static_cast<uint32>(IdPrefixSize + Body.Num()));
        WriteU32LE(Frame.GetData() + HeaderSize, Id);
        if (Body.Num() > 0)
        {
            FMemory::Memcpy(Frame.GetData() + HeaderSize + IdPrefixSize, Body.GetData(), Body.Num());
        }
        return Frame;
    }

    TArray<uint8> BuildJsonFrame(EOp Op, const FString& Json)
    {
        return BuildFrame(Op, ToUtf8(Json));
    }

    TArray<uint8> ToUtf8(const FString& Text)
    {
        const FTCHARToUTF8 Converter(*Text, Text.Len());
        return TArray<uint8>(reinterpret_cast<const uint8*>(Converter.Get()), Converter.Length());
    }

    FString FromUtf8(TConstArrayView<uint8> Utf8)
    {
        if (Utf8.Num() == 0)
        {
            return FString();
        }
        const FUTF8ToTCHAR Converter(reinterpret_cast<const ANSICHAR*>(Utf8.GetData()), Utf8.Num());
        return FString(Converter.Length(), Converter.Get());
    }

    FString DefaultClientName()
    {
        return FString::Printf(TEXT("uBot UE %s"), ClientVersion);
    }

    FString MakeHelloJson(const FString& ClientName)
    {
        FString Json;
        const TSharedRef<Private::FCompactJsonWriter> Writer = Private::FCompactJsonWriterFactory::Create(&Json);
        Writer->WriteObjectStart();
        Writer->WriteValue(TEXT("protocol"), ProtocolVersion);
        Writer->WriteValue(TEXT("client"), ClientName);
        Writer->WriteObjectEnd();
        Writer->Close();
        return Json;
    }

    FString MakeHelloAckJson(const FString& ServerName, const FString& RosDistro, int32 Protocol)
    {
        FString Json;
        const TSharedRef<Private::FCompactJsonWriter> Writer = Private::FCompactJsonWriterFactory::Create(&Json);
        Writer->WriteObjectStart();
        Writer->WriteValue(TEXT("protocol"), Protocol);
        Writer->WriteValue(TEXT("server"), ServerName);
        Writer->WriteValue(TEXT("ros_distro"), RosDistro);
        Writer->WriteObjectEnd();
        Writer->Close();
        return Json;
    }

    FString MakeQosJson(const FUBotROSQos& Qos)
    {
        FString Json;
        const TSharedRef<Private::FCompactJsonWriter> Writer = Private::FCompactJsonWriterFactory::Create(&Json);
        Writer->WriteObjectStart();
        Writer->WriteValue(TEXT("reliability"), Private::ReliabilityToString(Qos.Reliability));
        Writer->WriteValue(TEXT("durability"), Private::DurabilityToString(Qos.Durability));
        Writer->WriteValue(TEXT("depth"), FMath::Max(1, static_cast<int32>(Qos.Depth)));
        Writer->WriteObjectEnd();
        Writer->Close();
        return Json;
    }

    FString MakeTopicJson(uint32 Id, const FString& Topic, const FString& Type, const FUBotROSQos& Qos)
    {
        FString Json;
        const TSharedRef<Private::FCompactJsonWriter> Writer = Private::FCompactJsonWriterFactory::Create(&Json);
        Writer->WriteObjectStart();
        Writer->WriteValue(TEXT("id"), Id);
        Writer->WriteValue(TEXT("topic"), Topic);
        Writer->WriteValue(TEXT("type"), Type);
        Private::WriteQos(*Writer, Qos);
        Writer->WriteObjectEnd();
        Writer->Close();
        return Json;
    }

    FString MakeIdJson(uint32 Id)
    {
        FString Json;
        const TSharedRef<Private::FCompactJsonWriter> Writer = Private::FCompactJsonWriterFactory::Create(&Json);
        Writer->WriteObjectStart();
        Writer->WriteValue(TEXT("id"), Id);
        Writer->WriteObjectEnd();
        Writer->Close();
        return Json;
    }

    FString MakeSubCountJson(const TArray<TPair<uint32, int32>>& Counts)
    {
        FString Json;
        const TSharedRef<Private::FCompactJsonWriter> Writer = Private::FCompactJsonWriterFactory::Create(&Json);
        Writer->WriteObjectStart();
        Writer->WriteObjectStart(TEXT("counts"));
        for (const TPair<uint32, int32>& Count : Counts)
        {
            Writer->WriteValue(LexToString(Count.Key), Count.Value);
        }
        Writer->WriteObjectEnd();
        Writer->WriteObjectEnd();
        Writer->Close();
        return Json;
    }

    FString MakeErrorJson(TOptional<uint32> Id, const FString& Code, const FString& Message)
    {
        FString Json;
        const TSharedRef<Private::FCompactJsonWriter> Writer = Private::FCompactJsonWriterFactory::Create(&Json);
        Writer->WriteObjectStart();
        if (Id.IsSet())
        {
            Writer->WriteValue(TEXT("id"), Id.GetValue());
        }
        else
        {
            Writer->WriteNull(TEXT("id"));
        }
        Writer->WriteValue(TEXT("code"), Code);
        Writer->WriteValue(TEXT("message"), Message);
        Writer->WriteObjectEnd();
        Writer->Close();
        return Json;
    }

    bool ParseHelloAck(TConstArrayView<uint8> Payload, FHelloAck& Out)
    {
        const TSharedPtr<FJsonObject> Object = Private::ParseObject(Payload);
        int64 Protocol = 0;
        if (!Object.IsValid() || !Private::GetIntegerField(*Object, TEXT("protocol"), MIN_int32, MAX_int32, Protocol))
        {
            return false;
        }
        Out = FHelloAck();
        Out.Protocol = static_cast<int32>(Protocol);
        Object->TryGetStringField(TEXT("server"), Out.Server);
        Object->TryGetStringField(TEXT("ros_distro"), Out.RosDistro);
        return true;
    }

    bool ParseHello(TConstArrayView<uint8> Payload, FHello& Out)
    {
        const TSharedPtr<FJsonObject> Object = Private::ParseObject(Payload);
        int64 Protocol = 0;
        if (!Object.IsValid() || !Private::GetIntegerField(*Object, TEXT("protocol"), MIN_int32, MAX_int32, Protocol))
        {
            return false;
        }
        Out = FHello();
        Out.Protocol = static_cast<int32>(Protocol);
        Object->TryGetStringField(TEXT("client"), Out.Client);
        return true;
    }

    bool ParseTopicRequest(TConstArrayView<uint8> Payload, FTopicRequest& Out)
    {
        const TSharedPtr<FJsonObject> Object = Private::ParseObject(Payload);
        if (!Object.IsValid())
        {
            return false;
        }

        FTopicRequest Request;
        if (!Private::GetId(*Object, Request.Id)
            || !Object->TryGetStringField(TEXT("topic"), Request.Topic)
            || !Object->TryGetStringField(TEXT("type"), Request.Type))
        {
            return false;
        }

        const TSharedPtr<FJsonObject>* QosObject = nullptr;
        if (!Object->TryGetObjectField(TEXT("qos"), QosObject) || QosObject == nullptr || !QosObject->IsValid())
        {
            return false;
        }

        FString Reliability;
        FString Durability;
        int64 Depth = 0;
        if (!(*QosObject)->TryGetStringField(TEXT("reliability"), Reliability)
            || !(*QosObject)->TryGetStringField(TEXT("durability"), Durability)
            || !Private::GetIntegerField(**QosObject, TEXT("depth"), 0, MAX_int32, Depth))
        {
            return false;
        }

        if (Reliability == TEXT("reliable"))
        {
            Request.Qos.Reliability = EUBotROSReliability::Reliable;
        }
        else if (Reliability == TEXT("best_effort"))
        {
            Request.Qos.Reliability = EUBotROSReliability::BestEffort;
        }
        else
        {
            return false;
        }

        if (Durability == TEXT("volatile"))
        {
            Request.Qos.Durability = EUBotROSDurability::Volatile;
        }
        else if (Durability == TEXT("transient_local"))
        {
            Request.Qos.Durability = EUBotROSDurability::TransientLocal;
        }
        else
        {
            return false;
        }

        Request.Qos.Depth = static_cast<int32>(Depth);
        Out = MoveTemp(Request);
        return true;
    }

    bool ParseIdRequest(TConstArrayView<uint8> Payload, uint32& OutId)
    {
        const TSharedPtr<FJsonObject> Object = Private::ParseObject(Payload);
        return Object.IsValid() && Private::GetId(*Object, OutId);
    }

    bool ParseSubCount(TConstArrayView<uint8> Payload, TArray<TPair<uint32, int32>>& OutCounts)
    {
        OutCounts.Reset();
        const TSharedPtr<FJsonObject> Object = Private::ParseObject(Payload);
        if (!Object.IsValid())
        {
            return false;
        }
        const TSharedPtr<FJsonObject>* Counts = nullptr;
        if (!Object->TryGetObjectField(TEXT("counts"), Counts) || Counts == nullptr || !Counts->IsValid())
        {
            return false;
        }

        for (const TPair<FString, TSharedPtr<FJsonValue>>& Entry : (*Counts)->Values)
        {
            uint32 Id = 0;
            if (!Private::ParseDecimalId(Entry.Key, Id))
            {
                continue;
            }
            if (!Entry.Value.IsValid() || Entry.Value->Type != EJson::Number)
            {
                continue;
            }
            const double Number = Entry.Value->AsNumber();
            if (!FMath::IsFinite(Number) || Number < 0.0 || Number > static_cast<double>(MAX_int32) || FMath::RoundToDouble(Number) != Number)
            {
                continue;
            }
            OutCounts.Emplace(Id, static_cast<int32>(Number));
        }
        return true;
    }

    bool ParseError(TConstArrayView<uint8> Payload, FErrorInfo& Out)
    {
        const TSharedPtr<FJsonObject> Object = Private::ParseObject(Payload);
        if (!Object.IsValid())
        {
            return false;
        }

        Out = FErrorInfo();
        uint32 Id = 0;
        if (Private::GetId(*Object, Id))
        {
            Out.Id = Id;
        }
        Object->TryGetStringField(TEXT("code"), Out.Code);
        Object->TryGetStringField(TEXT("message"), Out.Message);
        return true;
    }

    bool SplitIdPayload(TConstArrayView<uint8> Payload, uint32& OutId, TConstArrayView<uint8>& OutBody)
    {
        if (Payload.Num() < IdPrefixSize)
        {
            return false;
        }
        OutId = ReadU32LE(Payload.GetData());
        OutBody = Payload.RightChop(IdPrefixSize);
        return true;
    }

    bool IsValidTopicName(const FString& Topic)
    {
        if (Topic.Len() < 2 || Topic.Len() > Private::MaxTopicLength || Topic[0] != TEXT('/') || Topic[Topic.Len() - 1] == TEXT('/'))
        {
            return false;
        }

        bool bTokenStart = true;
        for (int32 Index = 1; Index < Topic.Len(); ++Index)
        {
            const TCHAR Char = Topic[Index];
            if (Char == TEXT('/'))
            {
                if (bTokenStart)
                {
                    // Empty token ("//").
                    return false;
                }
                bTokenStart = true;
                continue;
            }
            if (!Private::IsIdentifierChar(Char))
            {
                return false;
            }
            if (bTokenStart && !Private::IsIdentifierStart(Char))
            {
                // Tokens must not start with a digit.
                return false;
            }
            bTokenStart = false;
        }
        return true;
    }

    bool IsValidMessageType(const FString& Type)
    {
        TArray<FString> Parts;
        Type.ParseIntoArray(Parts, TEXT("/"), /*InCullEmpty*/ false);
        return Parts.Num() == 3
            && Private::IsIdentifier(Parts[0])
            && Parts[1] == TEXT("msg")
            && Private::IsIdentifier(Parts[2]);
    }

    const TCHAR* ParseErrorToString(EParseError Error)
    {
        switch (Error)
        {
        case EParseError::None:
            return TEXT("none");
        case EParseError::BadMagic:
            return TEXT("bad magic");
        case EParseError::BadVersion:
            return TEXT("unsupported protocol version");
        case EParseError::Oversize:
            return TEXT("payload exceeds the size limit");
        default:
            return TEXT("unknown");
        }
    }

    FFrameParser::FFrameParser(uint32 InMaxPayloadBytes)
        : MaxPayloadBytes(InMaxPayloadBytes)
    {
    }

    void FFrameParser::Reset()
    {
        HeaderFill = 0;
        bHaveHeader = false;
        PayloadLength = 0;
        PayloadFill = 0;
        Current = FWireFrame();
        Ready.Empty();
        Error = EParseError::None;
    }

    void FFrameParser::SetMaxPayloadBytes(uint32 InMaxPayloadBytes)
    {
        MaxPayloadBytes = InMaxPayloadBytes;
    }

    bool FFrameParser::Feed(const uint8* Data, int32 Num)
    {
        if (Error != EParseError::None)
        {
            return false;
        }

        int32 Offset = 0;
        while (Offset < Num)
        {
            if (!bHaveHeader)
            {
                const int32 Take = FMath::Min(HeaderSize - HeaderFill, Num - Offset);
                FMemory::Memcpy(HeaderBytes + HeaderFill, Data + Offset, Take);
                HeaderFill += Take;
                Offset += Take;
                if (HeaderFill < HeaderSize)
                {
                    break;
                }

                if (FMemory::Memcmp(HeaderBytes, Magic, sizeof(Magic)) != 0)
                {
                    Error = EParseError::BadMagic;
                    return false;
                }
                if (HeaderBytes[4] != Version)
                {
                    Error = EParseError::BadVersion;
                    return false;
                }
                const uint32 Length = ReadU32LE(HeaderBytes + 8);
                if (Length > MaxPayloadBytes || Length > static_cast<uint32>(MAX_int32))
                {
                    Error = EParseError::Oversize;
                    return false;
                }

                bHaveHeader = true;
                PayloadLength = Length;
                PayloadFill = 0;
                Current = FWireFrame();
                Current.Op = HeaderBytes[5];
                Current.Flags = ReadU16LE(HeaderBytes + 6);
                Current.Payload.SetNumUninitialized(static_cast<int32>(Length));
                if (Length == 0)
                {
                    CompleteCurrentFrame();
                }
                continue;
            }

            const int32 Take = static_cast<int32>(FMath::Min<int64>(static_cast<int64>(PayloadLength - PayloadFill), static_cast<int64>(Num - Offset)));
            FMemory::Memcpy(Current.Payload.GetData() + PayloadFill, Data + Offset, Take);
            PayloadFill += static_cast<uint32>(Take);
            Offset += Take;
            if (PayloadFill == PayloadLength)
            {
                CompleteCurrentFrame();
            }
        }
        return true;
    }

    void FFrameParser::CompleteCurrentFrame()
    {
        Ready.EmplaceLast(MoveTemp(Current));
        Current = FWireFrame();
        bHaveHeader = false;
        HeaderFill = 0;
        PayloadLength = 0;
        PayloadFill = 0;
    }

    bool FFrameParser::PopFrame(FWireFrame& OutFrame)
    {
        return Ready.TryPopFirst(OutFrame);
    }

    int64 FFrameParser::GetPartialBytes() const
    {
        return bHaveHeader ? static_cast<int64>(HeaderSize) + PayloadFill : static_cast<int64>(HeaderFill);
    }
}
