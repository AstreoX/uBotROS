#include "UBotCdr.h"

#include "Containers/StringConv.h"
#include "HAL/UnrealMemory.h"

namespace UBotCdrPrivate
{
    /** Representation identifiers of the encapsulation header (first two bytes, big endian). */
    constexpr uint8 CdrBigEndianId = 0x00;
    constexpr uint8 CdrLittleEndianId = 0x01;

    /** Copies Count elements of ElementSize bytes, reversing the bytes of each element when bSwap. */
    void CopyElements(uint8* Destination, const uint8* Source, int32 Count, int32 ElementSize, bool bSwap)
    {
        const int64 NumBytes = static_cast<int64>(Count) * ElementSize;
        if (!bSwap || ElementSize == 1)
        {
            FMemory::Memcpy(Destination, Source, NumBytes);
            return;
        }

        for (int32 ElementIndex = 0; ElementIndex < Count; ++ElementIndex)
        {
            const uint8* SourceElement = Source + static_cast<int64>(ElementIndex) * ElementSize;
            uint8* DestinationElement = Destination + static_cast<int64>(ElementIndex) * ElementSize;
            for (int32 ByteIndex = 0; ByteIndex < ElementSize; ++ByteIndex)
            {
                DestinationElement[ByteIndex] = SourceElement[ElementSize - 1 - ByteIndex];
            }
        }
    }

    /** CDR data is little endian; only a big-endian host has to swap when writing. */
    constexpr bool WriterSwapsBytes()
    {
        return !PLATFORM_LITTLE_ENDIAN;
    }
}

// ---------------------------------------------------------------------------------------------
// FUBotCdrWriter

FUBotCdrWriter::FUBotCdrWriter(int32 InitialCapacity)
{
    Buffer.Reserve(FMath::Max(InitialCapacity, EncapsulationHeaderSize));
    WriteEncapsulationHeader();
}

void FUBotCdrWriter::WriteEncapsulationHeader()
{
    // Representation identifier CDR_LE (00 01) followed by options 00 00.
    static constexpr uint8 Header[EncapsulationHeaderSize] = { 0x00, UBotCdrPrivate::CdrLittleEndianId, 0x00, 0x00 };
    Buffer.Append(Header, EncapsulationHeaderSize);
}

void FUBotCdrWriter::Reserve(int64 AdditionalBytes)
{
    if (AdditionalBytes > 0)
    {
        const int64 Requested = Buffer.Num() + AdditionalBytes;
        Buffer.Reserve(static_cast<int32>(FMath::Min<int64>(Requested, MAX_int32)));
    }
}

void FUBotCdrWriter::Align(int32 Alignment)
{
    checkSlow(Alignment == 1 || Alignment == 2 || Alignment == 4 || Alignment == 8);
    if (Alignment <= 1)
    {
        return;
    }

    const int32 Remainder = GetBodyOffset() % Alignment;
    if (Remainder != 0)
    {
        Buffer.AddZeroed(Alignment - Remainder);
    }
}

void FUBotCdrWriter::WritePrimitive(const void* Value, int32 Size)
{
    Align(Size);
    const int32 Offset = Buffer.AddUninitialized(Size);
    UBotCdrPrivate::CopyElements(Buffer.GetData() + Offset, static_cast<const uint8*>(Value), 1, Size, UBotCdrPrivate::WriterSwapsBytes());
}

void FUBotCdrWriter::WritePrimitiveArray(const void* Values, int32 Count, int32 ElementSize)
{
    if (Count <= 0)
    {
        // Nothing is serialized for zero elements, so there is no alignment either (as Fast CDR does).
        return;
    }

    Align(ElementSize);
    const int64 NumBytes = static_cast<int64>(Count) * ElementSize;
    checkf(NumBytes <= static_cast<int64>(MAX_int32) - Buffer.Num(), TEXT("CDR message exceeds 2 GiB"));
    const int32 Offset = Buffer.AddUninitialized(static_cast<int32>(NumBytes));
    UBotCdrPrivate::CopyElements(Buffer.GetData() + Offset, static_cast<const uint8*>(Values), Count, ElementSize, UBotCdrPrivate::WriterSwapsBytes());
}

void FUBotCdrWriter::WriteBool(bool Value)
{
    WriteU8(Value ? uint8(1) : uint8(0));
}

void FUBotCdrWriter::WriteU8(uint8 Value)
{
    Buffer.Add(Value);
}

void FUBotCdrWriter::WriteI8(int8 Value)
{
    Buffer.Add(static_cast<uint8>(Value));
}

void FUBotCdrWriter::WriteU16(uint16 Value)
{
    WritePrimitive(&Value, sizeof(Value));
}

void FUBotCdrWriter::WriteI16(int16 Value)
{
    WritePrimitive(&Value, sizeof(Value));
}

void FUBotCdrWriter::WriteU32(uint32 Value)
{
    WritePrimitive(&Value, sizeof(Value));
}

void FUBotCdrWriter::WriteI32(int32 Value)
{
    WritePrimitive(&Value, sizeof(Value));
}

void FUBotCdrWriter::WriteU64(uint64 Value)
{
    WritePrimitive(&Value, sizeof(Value));
}

void FUBotCdrWriter::WriteI64(int64 Value)
{
    WritePrimitive(&Value, sizeof(Value));
}

void FUBotCdrWriter::WriteF32(float Value)
{
    static_assert(sizeof(float) == 4, "CDR float32 must be 4 bytes");
    WritePrimitive(&Value, sizeof(Value));
}

void FUBotCdrWriter::WriteF64(double Value)
{
    static_assert(sizeof(double) == 8, "CDR float64 must be 8 bytes");
    WritePrimitive(&Value, sizeof(Value));
}

void FUBotCdrWriter::WriteString(const FString& Value)
{
    const auto Utf8 = StringCast<UTF8CHAR>(*Value, Value.Len());
    const int32 NumBytes = Utf8.Length();
    WriteU32(static_cast<uint32>(NumBytes) + 1u);
    WriteBytes(Utf8.Get(), NumBytes);
    Buffer.Add(0);
}

void FUBotCdrWriter::WriteBytes(const void* Data, int32 NumBytes)
{
    if (NumBytes > 0)
    {
        checkf(NumBytes <= MAX_int32 - Buffer.Num(), TEXT("CDR message exceeds 2 GiB"));
        Buffer.Append(static_cast<const uint8*>(Data), NumBytes);
    }
}

void FUBotCdrWriter::WriteBytes(TConstArrayView<uint8> Bytes)
{
    WriteBytes(Bytes.GetData(), Bytes.Num());
}

void FUBotCdrWriter::WriteSequenceLength(int32 Count)
{
    WriteU32(static_cast<uint32>(FMath::Max(Count, 0)));
}

void FUBotCdrWriter::WriteU8Sequence(TConstArrayView<uint8> Values)
{
    WriteSequenceLength(Values.Num());
    WriteBytes(Values.GetData(), Values.Num());
}

void FUBotCdrWriter::WriteF32Sequence(TConstArrayView<float> Values)
{
    WriteSequenceLength(Values.Num());
    WritePrimitiveArray(Values.GetData(), Values.Num(), sizeof(float));
}

void FUBotCdrWriter::WriteF64Sequence(TConstArrayView<double> Values)
{
    WriteSequenceLength(Values.Num());
    WritePrimitiveArray(Values.GetData(), Values.Num(), sizeof(double));
}

void FUBotCdrWriter::WriteStringSequence(TConstArrayView<FString> Values)
{
    WriteSequenceLength(Values.Num());
    for (const FString& Value : Values)
    {
        WriteString(Value);
    }
}

void FUBotCdrWriter::WriteF32Array(TConstArrayView<float> Values)
{
    WritePrimitiveArray(Values.GetData(), Values.Num(), sizeof(float));
}

void FUBotCdrWriter::WriteF64Array(TConstArrayView<double> Values)
{
    WritePrimitiveArray(Values.GetData(), Values.Num(), sizeof(double));
}

TArray<uint8> FUBotCdrWriter::Finish()
{
    TArray<uint8> Result = MoveTemp(Buffer);
    Buffer.Reset();
    WriteEncapsulationHeader();
    return Result;
}

// ---------------------------------------------------------------------------------------------
// FUBotCdrReader

FUBotCdrReader::FUBotCdrReader(TConstArrayView<uint8> InData)
    : Data(InData)
{
    if (Data.Num() >= FUBotCdrWriter::EncapsulationHeaderSize && Data[0] == 0x00
        && (Data[1] == UBotCdrPrivate::CdrLittleEndianId || Data[1] == UBotCdrPrivate::CdrBigEndianId))
    {
        // Bytes 2 and 3 are encapsulation options; XCDR1 readers ignore them.
        bBigEndian = Data[1] == UBotCdrPrivate::CdrBigEndianId;
        bSwapBytes = bBigEndian == static_cast<bool>(PLATFORM_LITTLE_ENDIAN);
        Position = FUBotCdrWriter::EncapsulationHeaderSize;
        bOk = true;
    }
    else
    {
        Position = Data.Num();
        bOk = false;
    }
}

bool FUBotCdrReader::Fail()
{
    bOk = false;
    return false;
}

bool FUBotCdrReader::Align(int32 Alignment)
{
    if (!bOk)
    {
        return false;
    }
    if (Alignment <= 1)
    {
        return true;
    }

    const int32 Remainder = GetBodyOffset() % Alignment;
    if (Remainder == 0)
    {
        return true;
    }

    const int32 Padding = Alignment - Remainder;
    if (Padding > Data.Num() - Position)
    {
        return Fail();
    }
    Position += Padding;
    return true;
}

bool FUBotCdrReader::ReadPrimitive(void* OutValue, int32 Size)
{
    if (!Align(Size) || Size > Data.Num() - Position)
    {
        FMemory::Memzero(OutValue, Size);
        return Fail();
    }

    UBotCdrPrivate::CopyElements(static_cast<uint8*>(OutValue), Data.GetData() + Position, 1, Size, bSwapBytes);
    Position += Size;
    return true;
}

bool FUBotCdrReader::ReadPrimitiveArray(void* OutValues, int32 Count, int32 ElementSize)
{
    if (Count <= 0)
    {
        return bOk;
    }

    const int64 NumBytes = static_cast<int64>(Count) * ElementSize;
    if (!Align(ElementSize) || NumBytes > Data.Num() - Position)
    {
        FMemory::Memzero(OutValues, NumBytes);
        return Fail();
    }

    UBotCdrPrivate::CopyElements(static_cast<uint8*>(OutValues), Data.GetData() + Position, Count, ElementSize, bSwapBytes);
    Position += static_cast<int32>(NumBytes);
    return true;
}

bool FUBotCdrReader::ReadBool(bool& OutValue)
{
    uint8 Raw = 0;
    if (!ReadU8(Raw) || Raw > 1)
    {
        OutValue = false;
        return Fail();
    }
    OutValue = Raw != 0;
    return true;
}

bool FUBotCdrReader::ReadU8(uint8& OutValue)
{
    return ReadPrimitive(&OutValue, sizeof(OutValue));
}

bool FUBotCdrReader::ReadI8(int8& OutValue)
{
    return ReadPrimitive(&OutValue, sizeof(OutValue));
}

bool FUBotCdrReader::ReadU16(uint16& OutValue)
{
    return ReadPrimitive(&OutValue, sizeof(OutValue));
}

bool FUBotCdrReader::ReadI16(int16& OutValue)
{
    return ReadPrimitive(&OutValue, sizeof(OutValue));
}

bool FUBotCdrReader::ReadU32(uint32& OutValue)
{
    return ReadPrimitive(&OutValue, sizeof(OutValue));
}

bool FUBotCdrReader::ReadI32(int32& OutValue)
{
    return ReadPrimitive(&OutValue, sizeof(OutValue));
}

bool FUBotCdrReader::ReadU64(uint64& OutValue)
{
    return ReadPrimitive(&OutValue, sizeof(OutValue));
}

bool FUBotCdrReader::ReadI64(int64& OutValue)
{
    return ReadPrimitive(&OutValue, sizeof(OutValue));
}

bool FUBotCdrReader::ReadF32(float& OutValue)
{
    return ReadPrimitive(&OutValue, sizeof(OutValue));
}

bool FUBotCdrReader::ReadF64(double& OutValue)
{
    return ReadPrimitive(&OutValue, sizeof(OutValue));
}

bool FUBotCdrReader::ReadString(FString& OutValue)
{
    OutValue.Reset();

    uint32 Length = 0;
    if (!ReadU32(Length))
    {
        return false;
    }
    if (Length == 0)
    {
        // Not produced by ROS 2 writers, but some CDR implementations encode "" this way.
        return true;
    }
    if (Length > static_cast<uint32>(Data.Num() - Position))
    {
        return Fail();
    }

    const uint8* Bytes = Data.GetData() + Position;
    int32 NumCharacters = static_cast<int32>(Length);
    if (Bytes[NumCharacters - 1] == 0)
    {
        --NumCharacters;
    }
    OutValue = FString::ConstructFromPtrSize(reinterpret_cast<const UTF8CHAR*>(Bytes), NumCharacters);
    Position += static_cast<int32>(Length);
    return true;
}

bool FUBotCdrReader::ReadBytes(TArrayView<uint8> OutBytes)
{
    return ReadPrimitiveArray(OutBytes.GetData(), OutBytes.Num(), 1);
}

bool FUBotCdrReader::ReadSequenceLength(int32& OutCount, int32 MinElementBytes)
{
    OutCount = 0;

    uint32 Count = 0;
    if (!ReadU32(Count))
    {
        return false;
    }

    const int64 MinimumBytes = static_cast<int64>(Count) * FMath::Max(MinElementBytes, 1);
    if (Count > static_cast<uint32>(MAX_int32) || MinimumBytes > Data.Num() - Position)
    {
        return Fail();
    }

    OutCount = static_cast<int32>(Count);
    return true;
}

bool FUBotCdrReader::ReadU8Sequence(TArray<uint8>& OutValues)
{
    OutValues.Reset();
    int32 Count = 0;
    if (!ReadSequenceLength(Count, sizeof(uint8)))
    {
        return false;
    }

    OutValues.SetNumUninitialized(Count);
    if (!ReadPrimitiveArray(OutValues.GetData(), Count, sizeof(uint8)))
    {
        OutValues.Reset();
        return false;
    }
    return true;
}

bool FUBotCdrReader::ReadF32Sequence(TArray<float>& OutValues)
{
    OutValues.Reset();
    int32 Count = 0;
    if (!ReadSequenceLength(Count, sizeof(float)))
    {
        return false;
    }

    OutValues.SetNumUninitialized(Count);
    if (!ReadPrimitiveArray(OutValues.GetData(), Count, sizeof(float)))
    {
        OutValues.Reset();
        return false;
    }
    return true;
}

bool FUBotCdrReader::ReadF64Sequence(TArray<double>& OutValues)
{
    OutValues.Reset();
    int32 Count = 0;
    if (!ReadSequenceLength(Count, sizeof(double)))
    {
        return false;
    }

    OutValues.SetNumUninitialized(Count);
    if (!ReadPrimitiveArray(OutValues.GetData(), Count, sizeof(double)))
    {
        OutValues.Reset();
        return false;
    }
    return true;
}

bool FUBotCdrReader::ReadStringSequence(TArray<FString>& OutValues)
{
    OutValues.Reset();
    int32 Count = 0;
    // Every string carries at least its 4-byte length.
    if (!ReadSequenceLength(Count, sizeof(uint32)))
    {
        return false;
    }

    OutValues.Reserve(Count);
    for (int32 Index = 0; Index < Count; ++Index)
    {
        if (!ReadString(OutValues.AddDefaulted_GetRef()))
        {
            OutValues.Reset();
            return false;
        }
    }
    return true;
}

bool FUBotCdrReader::ReadF32Array(TArrayView<float> OutValues)
{
    return ReadPrimitiveArray(OutValues.GetData(), OutValues.Num(), sizeof(float));
}

bool FUBotCdrReader::ReadF64Array(TArrayView<double> OutValues)
{
    return ReadPrimitiveArray(OutValues.GetData(), OutValues.Num(), sizeof(double));
}
