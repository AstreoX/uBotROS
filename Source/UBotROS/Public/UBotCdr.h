#pragma once

#include "CoreMinimal.h"
#include "Containers/ArrayView.h"
#include "Containers/StaticArray.h"

/**
 * ROS 2 CDR encoding as used on the wire by the ROS 2 middleware (XCDR1, little endian):
 *  - every buffer starts with the 4-byte encapsulation header 00 01 00 00 (CDR_LE, options 0);
 *  - primitives are aligned to their size (bool/uint8 1, uint16 2, uint32/float 4, uint64/double 8),
 *    measured from the first byte after the encapsulation header; padding bytes are written as 0;
 *  - bool is one byte (0 or 1);
 *  - string = uint32 length including the terminating NUL, the UTF-8 bytes, then NUL;
 *  - sequence (T[]) = uint32 element count followed by the elements; an empty sequence has no
 *    element alignment;
 *  - fixed-size array (T[N]) = the N elements only;
 *  - nested messages are their fields in declaration order (no extra alignment of their own).
 *
 * The writer never fails (a message beyond 2 GiB asserts). The reader never reads past its buffer:
 * every Read* returns false once the data is exhausted or malformed, the error is sticky (IsOk()
 * stays false and all later reads fail) and failed reads leave their output zeroed / empty. The
 * reader skips alignment padding without checking it: rmw_fastrtps leaves padding uninitialized.
 */
class UBOTROS_API FUBotCdrWriter
{
public:
    /** Size of the encapsulation header that precedes the CDR body. */
    static constexpr int32 EncapsulationHeaderSize = 4;

    /** Starts a buffer with the CDR_LE encapsulation header. InitialCapacity is a reservation hint. */
    explicit FUBotCdrWriter(int32 InitialCapacity = 256);

    /** Reserves room for at least AdditionalBytes more bytes (call before large payloads). */
    void Reserve(int64 AdditionalBytes);

    /** Pads with zero bytes until the body offset is a multiple of Alignment (1, 2, 4 or 8). */
    void Align(int32 Alignment);

    void WriteBool(bool Value);
    void WriteU8(uint8 Value);
    void WriteI8(int8 Value);
    void WriteU16(uint16 Value);
    void WriteI16(int16 Value);
    void WriteU32(uint32 Value);
    void WriteI32(int32 Value);
    void WriteU64(uint64 Value);
    void WriteI64(int64 Value);
    void WriteF32(float Value);
    void WriteF64(double Value);

    /** ROS string: UTF-8 encoded, uint32 length including NUL, bytes, NUL. */
    void WriteString(const FString& Value);

    /** Raw octets: no alignment and no length prefix (e.g. a pre-encoded nested payload). */
    void WriteBytes(const void* Data, int32 NumBytes);
    void WriteBytes(TConstArrayView<uint8> Bytes);

    /** uint32 element count of a sequence; negative counts are written as 0. */
    void WriteSequenceLength(int32 Count);

    /** uint8[] / byte[]: count + bytes. */
    void WriteU8Sequence(TConstArrayView<uint8> Values);

    /** float32[]: count + elements. */
    void WriteF32Sequence(TConstArrayView<float> Values);

    /** float64[]: count + elements. */
    void WriteF64Sequence(TConstArrayView<double> Values);

    /** string[]: count + strings. */
    void WriteStringSequence(TConstArrayView<FString> Values);

    /** float32[N]: elements only. */
    void WriteF32Array(TConstArrayView<float> Values);

    /** float64[N]: elements only. */
    void WriteF64Array(TConstArrayView<double> Values);

    template <uint32 N>
    void WriteF64Array(const TStaticArray<double, N>& Values)
    {
        WriteF64Array(TConstArrayView<double>(Values.GetData(), static_cast<int32>(N)));
    }

    /** Bytes written so far, including the encapsulation header. */
    int32 Num() const
    {
        return Buffer.Num();
    }

    /** Bytes after the encapsulation header; alignment is relative to this offset. */
    int32 GetBodyOffset() const
    {
        return Buffer.Num() - EncapsulationHeaderSize;
    }

    /** The bytes written so far, including the encapsulation header. */
    TConstArrayView<uint8> GetData() const
    {
        return Buffer;
    }

    /** Returns the complete serialized message and resets the writer to an empty message. */
    TArray<uint8> Finish();

private:
    void WriteEncapsulationHeader();
    void WritePrimitive(const void* Value, int32 Size);
    void WritePrimitiveArray(const void* Values, int32 Count, int32 ElementSize);

    TArray<uint8> Buffer;
};

class UBOTROS_API FUBotCdrReader
{
public:
    /**
     * Reads a serialized message (encapsulation header + CDR body). The data is not copied and must
     * outlive the reader. CDR_LE and CDR_BE encapsulations are accepted; anything else (or fewer than 4
     * bytes) leaves the reader in the failed state. Trailing bytes after the last read are ignored.
     */
    explicit FUBotCdrReader(TConstArrayView<uint8> InData);

    /** False once the header was invalid or any read failed. */
    bool IsOk() const
    {
        return bOk;
    }

    /** True when the encapsulation header announced big-endian data (byte-swapped transparently). */
    bool IsBigEndian() const
    {
        return bBigEndian;
    }

    /** Current position relative to the first byte after the encapsulation header. */
    int32 GetBodyOffset() const
    {
        return Position - FUBotCdrWriter::EncapsulationHeaderSize;
    }

    /** Bytes left after the current position (0 when failed). */
    int32 GetRemaining() const
    {
        return bOk ? Data.Num() - Position : 0;
    }

    /** Puts the reader into the failed state, e.g. when a decoded value is semantically invalid. */
    void SetError()
    {
        bOk = false;
    }

    /** Skips padding until the body offset is a multiple of Alignment. Padding content is not checked. */
    bool Align(int32 Alignment);

    /** Accepts 0 and 1 only. */
    bool ReadBool(bool& OutValue);
    bool ReadU8(uint8& OutValue);
    bool ReadI8(int8& OutValue);
    bool ReadU16(uint16& OutValue);
    bool ReadI16(int16& OutValue);
    bool ReadU32(uint32& OutValue);
    bool ReadI32(int32& OutValue);
    bool ReadU64(uint64& OutValue);
    bool ReadI64(int64& OutValue);
    bool ReadF32(float& OutValue);
    bool ReadF64(double& OutValue);

    /** ROS string. A length of 0 is accepted as empty; one trailing NUL is stripped; UTF-8 decoded. */
    bool ReadString(FString& OutValue);

    /** Raw octets: no alignment and no length prefix. Fills OutBytes completely or fails. */
    bool ReadBytes(TArrayView<uint8> OutBytes);

    /**
     * uint32 sequence count. Fails when the remaining data cannot hold Count elements of at least
     * MinElementBytes each, which bounds allocations driven by corrupt input.
     */
    bool ReadSequenceLength(int32& OutCount, int32 MinElementBytes = 1);

    bool ReadU8Sequence(TArray<uint8>& OutValues);
    bool ReadF32Sequence(TArray<float>& OutValues);
    bool ReadF64Sequence(TArray<double>& OutValues);
    bool ReadStringSequence(TArray<FString>& OutValues);

    /** float32[N] / float64[N]: fills OutValues completely (elements only) or fails. */
    bool ReadF32Array(TArrayView<float> OutValues);
    bool ReadF64Array(TArrayView<double> OutValues);

    template <uint32 N>
    bool ReadF64Array(TStaticArray<double, N>& OutValues)
    {
        return ReadF64Array(TArrayView<double>(OutValues.GetData(), static_cast<int32>(N)));
    }

private:
    bool ReadPrimitive(void* OutValue, int32 Size);
    bool ReadPrimitiveArray(void* OutValues, int32 Count, int32 ElementSize);
    bool Fail();

    TConstArrayView<uint8> Data;
    int32 Position = 0;
    bool bOk = false;
    bool bBigEndian = false;
    bool bSwapBytes = false;
};
