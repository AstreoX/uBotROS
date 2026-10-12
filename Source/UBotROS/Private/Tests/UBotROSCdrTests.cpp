#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"
#include "UBotCdr.h"
#include "UBotROSMessageTypes.h"
#include "UBotROSTypes.h"

#include <limits>

#if WITH_DEV_AUTOMATION_TESTS

#include "UBotROSGoldenData.h"

namespace UBotROSCdrTests
{
    constexpr EAutomationTestFlags TestFlags = EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter;

    // Layout characters of UBotROSGoldenData (see ros2/tools/gen_golden.py).
    constexpr ANSICHAR LayoutPadding = 'p';
    constexpr ANSICHAR LayoutTrailing = 't';

    /** A golden reference: the bytes produced by rclpy and one layout character per byte. */
    struct FGoldenCase
    {
        TConstArrayView<uint8> Bytes;
        const ANSICHAR* Layout = nullptr;

        /** Length of the CDR stream proper (the reference without rmw trailing size padding). */
        int32 GetCdrSize() const
        {
            int32 Size = Bytes.Num();
            while (Size > 0 && Layout[Size - 1] == LayoutTrailing)
            {
                --Size;
            }
            return Size;
        }
    };

    template <SIZE_T NumBytes, SIZE_T NumLayoutCharacters>
    FGoldenCase MakeGolden(const uint8 (&Bytes)[NumBytes], const ANSICHAR (&Layout)[NumLayoutCharacters])
    {
        static_assert(NumLayoutCharacters == NumBytes + 1, "The golden layout needs one character per byte");
        FGoldenCase Result;
        Result.Bytes = TConstArrayView<uint8>(Bytes, static_cast<int32>(NumBytes));
        Result.Layout = Layout;
        return Result;
    }

    FString ToHex(TConstArrayView<uint8> Bytes)
    {
        FString Result;
        Result.Reserve(Bytes.Num() * 3);
        for (int32 Index = 0; Index < Bytes.Num(); ++Index)
        {
            if (Index > 0)
            {
                Result.AppendChar(TEXT(' '));
            }
            Result += FString::Printf(TEXT("%02X"), Bytes[Index]);
        }
        return Result;
    }

    /** Compares Actual with an expected byte string and reports the first differences. */
    bool ExpectBytes(FAutomationTestBase& Test, const FString& What, TConstArrayView<uint8> Actual, TConstArrayView<uint8> Expected)
    {
        if (Actual.Num() != Expected.Num())
        {
            Test.AddError(FString::Printf(TEXT("%s: %d bytes, expected %d.\n  actual:   %s\n  expected: %s"),
                *What, Actual.Num(), Expected.Num(), *ToHex(Actual), *ToHex(Expected)));
            return false;
        }
        for (int32 Index = 0; Index < Actual.Num(); ++Index)
        {
            if (Actual[Index] != Expected[Index])
            {
                Test.AddError(FString::Printf(TEXT("%s: first difference at byte %d.\n  actual:   %s\n  expected: %s"),
                    *What, Index, *ToHex(Actual), *ToHex(Expected)));
                return false;
            }
        }
        return true;
    }

    /**
     * Byte-for-byte comparison with a golden reference. Alignment padding must be zero in Actual (the
     * reference holds whatever rmw_fastrtps left there); rmw trailing size padding is not expected.
     */
    bool ExpectGolden(FAutomationTestBase& Test, const FString& What, TConstArrayView<uint8> Actual, const FGoldenCase& Golden)
    {
        const int32 CdrSize = Golden.GetCdrSize();
        if (Actual.Num() != CdrSize)
        {
            Test.AddError(FString::Printf(TEXT("%s: %d bytes, the reference CDR stream has %d.\n  actual:    %s\n  reference: %s"),
                *What, Actual.Num(), CdrSize, *ToHex(Actual), *ToHex(Golden.Bytes)));
            return false;
        }

        constexpr int32 MaxReportedDifferences = 8;
        int32 NumDifferences = 0;
        for (int32 Index = 0; Index < CdrSize; ++Index)
        {
            const bool bPadding = Golden.Layout[Index] == LayoutPadding;
            uint8 Expected = 0;
            if (!bPadding)
            {
                Expected = Golden.Bytes[Index];
            }
            if (Actual[Index] != Expected)
            {
                if (NumDifferences < MaxReportedDifferences)
                {
                    Test.AddError(FString::Printf(TEXT("%s: byte %d is 0x%02X, expected 0x%02X (%s)."),
                        *What, Index, Actual[Index], Expected, bPadding ? TEXT("alignment padding") : TEXT("data")));
                }
                ++NumDifferences;
            }
        }
        if (NumDifferences > 0)
        {
            Test.AddError(FString::Printf(TEXT("%s: %d differing bytes.\n  actual:    %s\n  reference: %s"),
                *What, NumDifferences, *ToHex(Actual), *ToHex(Golden.Bytes)));
        }
        return NumDifferences == 0;
    }

    /**
     * The golden test of one message type: the type name, the encoding of Message against the rclpy
     * reference, and decoding the reference (with its arbitrary padding bytes) back to the same bytes.
     */
    template <typename T>
    bool CheckGolden(FAutomationTestBase& Test, const T& Message, const FGoldenCase& Golden, const TCHAR* ExpectedTypeName)
    {
        Test.TestEqual(TEXT("Type name"), T::TypeName(), ExpectedTypeName);

        const TArray<uint8> Encoded = UBot::Ros::Encode(Message);
        const bool bEncodeMatches = ExpectGolden(Test, TEXT("Encode"), Encoded, Golden);

        T Decoded;
        const bool bDecoded = UBot::Ros::Decode(Golden.Bytes, Decoded);
        Test.TestTrue(TEXT("Decode of the rclpy reference succeeds"), bDecoded);
        bool bRoundTripMatches = false;
        if (bDecoded)
        {
            bRoundTripMatches = ExpectGolden(Test, TEXT("Decode + encode of the reference"), UBot::Ros::Encode(Decoded), Golden);
        }

        // Every truncation of the CDR stream must be rejected without reading past the buffer.
        const int32 CdrSize = Golden.GetCdrSize();
        int32 NumAcceptedTruncations = 0;
        for (int32 Length = 0; Length < CdrSize; ++Length)
        {
            T Truncated;
            if (UBot::Ros::Decode(Golden.Bytes.Left(Length), Truncated))
            {
                ++NumAcceptedTruncations;
            }
        }
        Test.TestEqual(TEXT("Truncated references accepted by Decode"), NumAcceptedTruncations, 0);

        return bEncodeMatches && bDecoded && bRoundTripMatches && NumAcceptedTruncations == 0;
    }

    template <typename T>
    uint64 Bits(T Value)
    {
        static_assert(sizeof(T) <= sizeof(uint64), "Bits supports types up to 8 bytes");
        uint64 Result = 0;
        FMemory::Memcpy(&Result, &Value, sizeof(T));
        return Result;
    }

    /** Exact equality including the sign of zero, infinities and NaN payloads. */
    template <typename T>
    bool ExpectSameBits(FAutomationTestBase& Test, const FString& What, T Actual, T Expected)
    {
        if (Bits(Actual) != Bits(Expected))
        {
            Test.AddError(FString::Printf(TEXT("%s: %.17g, expected %.17g."), *What, static_cast<double>(Actual), static_cast<double>(Expected)));
            return false;
        }
        return true;
    }

    TArray<uint8> WithHeader(std::initializer_list<uint8> Body)
    {
        TArray<uint8> Result = { 0x00, 0x01, 0x00, 0x00 };
        Result.Append(Body);
        return Result;
    }

    // ---------------------------------------------------------------------------------------------
    // Messages with the values of ros2/tools/gen_golden.py. Keep both in sync.

    FUBotRosTime MakeTime(int32 Sec, uint32 Nanosec)
    {
        FUBotRosTime Result;
        Result.sec = Sec;
        Result.nanosec = Nanosec;
        return Result;
    }

    FUBotRosHeader MakeHeader(int32 Sec, uint32 Nanosec, const FString& FrameId)
    {
        FUBotRosHeader Result;
        Result.stamp = MakeTime(Sec, Nanosec);
        Result.frame_id = FrameId;
        return Result;
    }

    FUBotRosVector3 MakeVector3(double X, double Y, double Z)
    {
        FUBotRosVector3 Result;
        Result.x = X;
        Result.y = Y;
        Result.z = Z;
        return Result;
    }

    FUBotRosQuaternion MakeQuaternion(double X, double Y, double Z, double W)
    {
        FUBotRosQuaternion Result;
        Result.x = X;
        Result.y = Y;
        Result.z = Z;
        Result.w = W;
        return Result;
    }

    FUBotRosTransformStamped MakeTransformStamped(const FUBotRosHeader& Header, const FString& ChildFrameId, const FUBotRosVector3& Translation, const FUBotRosQuaternion& Rotation)
    {
        FUBotRosTransformStamped Result;
        Result.header = Header;
        Result.child_frame_id = ChildFrameId;
        Result.transform.translation = Translation;
        Result.transform.rotation = Rotation;
        return Result;
    }

    FUBotRosPoseStamped MakePoseStamped()
    {
        FUBotRosPoseStamped Result;
        Result.header = MakeHeader(2, 3, TEXT("ab"));
        Result.pose.position.x = -1.0;
        Result.pose.position.y = 2.0;
        Result.pose.position.z = -3.0;
        Result.pose.orientation = MakeQuaternion(0.5, 0.5, -0.5, 0.5);
        return Result;
    }

    FUBotRosTransformStamped MakeSingleTransformStamped()
    {
        return MakeTransformStamped(MakeHeader(4, 5, TEXT("odom")), TEXT("base_footprint"),
            MakeVector3(3.5, -4.25, 0.0), MakeQuaternion(0.0, 0.0, -0.5, 0.8660254037844386));
    }

    FUBotRosImu MakeImu()
    {
        FUBotRosImu Result;
        Result.header = MakeHeader(100, 250000000, TEXT("imu_frame"));
        Result.orientation = MakeQuaternion(0.125, -0.25, 0.375, 0.875);
        Result.angular_velocity = MakeVector3(0.1, -0.2, 0.3);
        Result.linear_acceleration = MakeVector3(-1.5, 2.25, 9.81);
        for (int32 Index = 0; Index < 9; ++Index)
        {
            Result.orientation_covariance[Index] = 1.0 + Index * 0.125;
            Result.angular_velocity_covariance[Index] = 2.0 + Index * 0.125;
            Result.linear_acceleration_covariance[Index] = 3.0 + Index * 0.125;
        }
        return Result;
    }

    FUBotRosOdometry MakeOdometry()
    {
        FUBotRosOdometry Result;
        Result.header = MakeHeader(7, 123456789, TEXT("world"));
        Result.child_frame_id = TEXT("base_link");
        Result.pose.pose.position.x = 1.25;
        Result.pose.pose.position.y = -2.5;
        Result.pose.pose.position.z = 3.75;
        Result.pose.pose.orientation = MakeQuaternion(0.0, 0.0, 0.38268343236508984, 0.9238795325112867);
        Result.twist.twist.linear = MakeVector3(0.5, -0.125, 0.0625);
        Result.twist.twist.angular = MakeVector3(-0.03125, 0.015625, 0.75);
        for (int32 Index = 0; Index < 36; ++Index)
        {
            Result.pose.covariance[Index] = Index * 0.5;
            // Python's -(0 * 0.25) is -0.0; so is this one.
            Result.twist.covariance[Index] = -(Index * 0.25);
        }
        return Result;
    }

    FUBotRosLaserScan MakeLaserScan()
    {
        FUBotRosLaserScan Result;
        Result.header = MakeHeader(3, 999999999, TEXT("laser"));
        Result.angle_min = -1.5f;
        Result.angle_max = 1.5f;
        Result.angle_increment = 0.75f;
        Result.time_increment = 0.0009765625f;
        Result.scan_time = 0.0625f;
        Result.range_min = 0.125f;
        Result.range_max = 32.0f;
        Result.ranges = { 1.0f, 2.5f, std::numeric_limits<float>::infinity(), 0.5f, 12.25f };
        Result.intensities = { 100.0f, 0.0f, 50.5f, 7.75f, 1.0f };
        return Result;
    }

    FUBotRosPointField MakeSinglePointField()
    {
        return FUBotRosPointField::Make(TEXT("intensity"), 12, FUBotRosPointField::FLOAT32, 1);
    }

    void AppendLittleEndian(TArray<uint8>& Bytes, const void* Value, int32 Size)
    {
        // The golden data is little endian; every UE target platform is little endian as well.
        static_assert(PLATFORM_LITTLE_ENDIAN != 0, "AppendLittleEndian assumes a little-endian host");
        Bytes.Append(static_cast<const uint8*>(Value), Size);
    }

    FUBotRosPointCloud2 MakePointCloud2()
    {
        FUBotRosPointCloud2 Result;
        Result.header = MakeHeader(5, 5, TEXT("lidar_top"));
        Result.height = 1;
        Result.width = 3;
        Result.fields.Add(FUBotRosPointField::Make(TEXT("x"), 0, FUBotRosPointField::FLOAT32, 1));
        Result.fields.Add(FUBotRosPointField::Make(TEXT("ring"), 4, FUBotRosPointField::UINT16, 1));
        Result.is_bigendian = false;
        Result.point_step = 6;
        Result.row_step = 18;
        const float X[3] = { 1.5f, -2.25f, 3.0f };
        for (int32 Index = 0; Index < 3; ++Index)
        {
            const uint16 Ring = static_cast<uint16>(Index);
            AppendLittleEndian(Result.data, &X[Index], sizeof(float));
            AppendLittleEndian(Result.data, &Ring, sizeof(uint16));
        }
        Result.is_dense = true;
        return Result;
    }

    FUBotRosImage MakeImage()
    {
        FUBotRosImage Result;
        Result.header = MakeHeader(9, 87654321, TEXT("cam_optical"));
        Result.height = 2;
        Result.width = 3;
        Result.encoding = TEXT("rgb8");
        Result.is_bigendian = 0;
        Result.step = 9;
        for (int32 Index = 0; Index < 18; ++Index)
        {
            Result.data.Add(static_cast<uint8>((Index * 37 + 11) & 0xFF));
        }
        return Result;
    }

    FUBotRosRegionOfInterest MakeRegionOfInterest(uint32 XOffset, uint32 YOffset, uint32 Height, uint32 Width, bool bDoRectify)
    {
        FUBotRosRegionOfInterest Result;
        Result.x_offset = XOffset;
        Result.y_offset = YOffset;
        Result.height = Height;
        Result.width = Width;
        Result.do_rectify = bDoRectify;
        return Result;
    }

    FUBotRosCameraInfo MakeCameraInfo()
    {
        FUBotRosCameraInfo Result;
        Result.header = MakeHeader(9, 87654321, TEXT("cam_optical"));
        Result.height = 480;
        Result.width = 640;
        Result.distortion_model = TEXT("plumb_bob");
        Result.d = { 0.125, -0.25, 0.0009765625, -0.001953125, 0.5 };
        const double K[9] = { 500.0, 0.0, 320.5, 0.0, 501.0, 240.5, 0.0, 0.0, 1.0 };
        const double R[9] = { 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 };
        const double P[12] = { 500.0, 0.0, 320.5, -35.0, 0.0, 501.0, 240.5, 0.0, 0.0, 0.0, 1.0, 0.0 };
        for (int32 Index = 0; Index < 9; ++Index)
        {
            Result.k[Index] = K[Index];
            Result.r[Index] = R[Index];
        }
        for (int32 Index = 0; Index < 12; ++Index)
        {
            Result.p[Index] = P[Index];
        }
        Result.binning_x = 1;
        Result.binning_y = 2;
        Result.roi = MakeRegionOfInterest(10, 20, 30, 40, true);
        return Result;
    }

    FUBotRosTfMessage MakeTfMessage()
    {
        FUBotRosTfMessage Result;
        Result.transforms.Add(MakeTransformStamped(MakeHeader(11, 1, TEXT("world")), TEXT("base_link"),
            MakeVector3(1.0, -2.0, 0.5), MakeQuaternion(0.0, 0.0, 0.0, 1.0)));
        Result.transforms.Add(MakeTransformStamped(MakeHeader(11, 1, TEXT("base_link")), TEXT("laser"),
            MakeVector3(0.25, 0.0, 0.125), MakeQuaternion(0.0, 0.0, 0.7071067811865476, 0.7071067811865476)));
        return Result;
    }

    FUBotRosClock MakeClock()
    {
        FUBotRosClock Result;
        Result.clock = MakeTime(98765, 4321);
        return Result;
    }

    FUBotRosTwist MakeTwist()
    {
        FUBotRosTwist Result;
        Result.linear = MakeVector3(1.5, -0.25, 0.125);
        Result.angular = MakeVector3(-0.5, 0.75, 0.3);
        return Result;
    }

    FUBotRosAttitudeTarget MakeAttitudeTarget()
    {
        FUBotRosAttitudeTarget Result;
        Result.header = MakeHeader(3, 250000000, TEXT("map"));
        Result.type_mask = FUBotRosAttitudeTarget::IGNORE_ROLL_RATE | FUBotRosAttitudeTarget::IGNORE_PITCH_RATE;
        Result.orientation = MakeQuaternion(0.1, -0.2, 0.3, 0.9);
        Result.body_rate = MakeVector3(0.5, -0.25, 1.5);
        Result.thrust = 0.625f;
        return Result;
    }

    FUBotRosActuators MakeActuators()
    {
        FUBotRosActuators Result;
        Result.header = MakeHeader(7, 5, TEXT("base_link"));
        Result.position = { 0.5 };
        Result.velocity = { 800.0, 810.5, 0.0, 795.25 };
        Result.normalized = { 0.25, 0.5, 0.75, 1.0 };
        return Result;
    }

    bool ExpectSameTwist(FAutomationTestBase& Test, const FString& What, const FUBotRosTwist& Actual, const FUBotRosTwist& Expected)
    {
        // Every component is checked (and reported) even after a mismatch.
        int32 NumDifferent = 0;
        NumDifferent += ExpectSameBits(Test, What + TEXT(" linear.x"), Actual.linear.x, Expected.linear.x) ? 0 : 1;
        NumDifferent += ExpectSameBits(Test, What + TEXT(" linear.y"), Actual.linear.y, Expected.linear.y) ? 0 : 1;
        NumDifferent += ExpectSameBits(Test, What + TEXT(" linear.z"), Actual.linear.z, Expected.linear.z) ? 0 : 1;
        NumDifferent += ExpectSameBits(Test, What + TEXT(" angular.x"), Actual.angular.x, Expected.angular.x) ? 0 : 1;
        NumDifferent += ExpectSameBits(Test, What + TEXT(" angular.y"), Actual.angular.y, Expected.angular.y) ? 0 : 1;
        NumDifferent += ExpectSameBits(Test, What + TEXT(" angular.z"), Actual.angular.z, Expected.angular.z) ? 0 : 1;
        return NumDifferent == 0;
    }
}

// -------------------------------------------------------------------------------------------------
// Writer / reader basics

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotROSCdrPrimitivesTest, "UBotROS.Cdr.Primitives", UBotROSCdrTests::TestFlags)

bool FUBotROSCdrPrimitivesTest::RunTest(const FString& Parameters)
{
    using namespace UBotROSCdrTests;

    FUBotCdrWriter Writer;
    TestEqual(TEXT("A new writer holds only the encapsulation header"), Writer.Num(), FUBotCdrWriter::EncapsulationHeaderSize);
    Writer.WriteU8(0xAB);
    Writer.WriteU16(0x1234);
    Writer.WriteU32(0xDEADBEEF);
    Writer.WriteU64(0x0102030405060708ull);
    Writer.WriteF32(1.0f);
    Writer.WriteF64(-2.0);
    Writer.WriteBool(true);
    Writer.WriteI8(-2);
    Writer.WriteI16(-3);
    Writer.WriteI32(-4);
    Writer.WriteI64(-5);
    const TArray<uint8> Encoded = Writer.Finish();

    const TArray<uint8> Expected = WithHeader({
        0xAB,                                           // body 0: uint8
        0x00,                                           // pad to 2
        0x34, 0x12,                                     // body 2: uint16
        0xEF, 0xBE, 0xAD, 0xDE,                         // body 4: uint32
        0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01, // body 8: uint64
        0x00, 0x00, 0x80, 0x3F,                         // body 16: float32 1.0
        0x00, 0x00, 0x00, 0x00,                         // pad to 24
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, // body 24: float64 -2.0
        0x01,                                           // body 32: bool
        0xFE,                                           // body 33: int8 -2
        0xFD, 0xFF,                                     // body 34: int16 -3
        0xFC, 0xFF, 0xFF, 0xFF,                         // body 36: int32 -4
        0xFB, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, // body 40: int64 -5
    });
    ExpectBytes(*this, TEXT("Encoded primitives"), Encoded, Expected);
    TestEqual(TEXT("Finish resets the writer to the encapsulation header"), Writer.Num(), FUBotCdrWriter::EncapsulationHeaderSize);

    FUBotCdrReader Reader(Encoded);
    TestTrue(TEXT("Reader accepts the CDR_LE header"), Reader.IsOk());
    TestFalse(TEXT("CDR_LE is not big endian"), Reader.IsBigEndian());

    uint8 U8 = 0;
    uint16 U16 = 0;
    uint32 U32 = 0;
    uint64 U64 = 0;
    float F32 = 0.0f;
    double F64 = 0.0;
    bool bBool = false;
    int8 I8 = 0;
    int16 I16 = 0;
    int32 I32 = 0;
    int64 I64 = 0;
    TestTrue(TEXT("Read uint8"), Reader.ReadU8(U8) && U8 == 0xAB);
    TestTrue(TEXT("Read uint16"), Reader.ReadU16(U16) && U16 == 0x1234);
    TestTrue(TEXT("Read uint32"), Reader.ReadU32(U32) && U32 == 0xDEADBEEF);
    TestTrue(TEXT("Read uint64"), Reader.ReadU64(U64) && U64 == 0x0102030405060708ull);
    TestTrue(TEXT("Read float32"), Reader.ReadF32(F32) && F32 == 1.0f);
    TestTrue(TEXT("Read float64"), Reader.ReadF64(F64) && F64 == -2.0);
    TestTrue(TEXT("Read bool"), Reader.ReadBool(bBool) && bBool);
    TestTrue(TEXT("Read int8"), Reader.ReadI8(I8) && I8 == -2);
    TestTrue(TEXT("Read int16"), Reader.ReadI16(I16) && I16 == -3);
    TestTrue(TEXT("Read int32"), Reader.ReadI32(I32) && I32 == -4);
    TestTrue(TEXT("Read int64"), Reader.ReadI64(I64) && I64 == -5);
    TestTrue(TEXT("Reader still ok"), Reader.IsOk());
    TestEqual(TEXT("Everything consumed"), Reader.GetRemaining(), 0);

    // Special floating point values survive bit for bit.
    FUBotCdrWriter SpecialWriter;
    SpecialWriter.WriteF32(-0.0f);
    SpecialWriter.WriteF32(std::numeric_limits<float>::infinity());
    SpecialWriter.WriteF64(-std::numeric_limits<double>::infinity());
    SpecialWriter.WriteF64(std::numeric_limits<double>::denorm_min());
    const TArray<uint8> SpecialEncoded = SpecialWriter.Finish();
    FUBotCdrReader SpecialReader(SpecialEncoded);
    float NegativeZero = 1.0f;
    float PositiveInfinity = 0.0f;
    double NegativeInfinity = 0.0;
    double Denormal = 0.0;
    SpecialReader.ReadF32(NegativeZero);
    SpecialReader.ReadF32(PositiveInfinity);
    SpecialReader.ReadF64(NegativeInfinity);
    SpecialReader.ReadF64(Denormal);
    TestTrue(TEXT("Special values read"), SpecialReader.IsOk());
    ExpectSameBits(*this, TEXT("-0.0f"), NegativeZero, -0.0f);
    ExpectSameBits(*this, TEXT("+inf f32"), PositiveInfinity, std::numeric_limits<float>::infinity());
    ExpectSameBits(*this, TEXT("-inf f64"), NegativeInfinity, -std::numeric_limits<double>::infinity());
    ExpectSameBits(*this, TEXT("denormal f64"), Denormal, std::numeric_limits<double>::denorm_min());
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotROSCdrAlignmentTest, "UBotROS.Cdr.Alignment", UBotROSCdrTests::TestFlags)

bool FUBotROSCdrAlignmentTest::RunTest(const FString& Parameters)
{
    using namespace UBotROSCdrTests;

    // Alignment is measured from the first byte after the 4-byte encapsulation header: a float64
    // after one uint8 starts at body offset 8 (absolute offset 12), not at absolute offset 8.
    {
        FUBotCdrWriter Writer;
        Writer.WriteU8(0x11);
        Writer.WriteF64(1.0);
        ExpectBytes(*this, TEXT("uint8 then float64"), Writer.Finish(), WithHeader({
            0x11, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF0, 0x3F }));
    }
    {
        FUBotCdrWriter Writer;
        Writer.WriteU8(0x11);
        Writer.WriteU16(0x2233);
        Writer.WriteU8(0x44);
        Writer.WriteU32(0x55667788);
        Writer.WriteU8(0x99);
        Writer.WriteU64(0x0102030405060708ull);
        ExpectBytes(*this, TEXT("Mixed alignment"), Writer.Finish(), WithHeader({
            0x11, 0x00, 0x33, 0x22, 0x44, 0x00, 0x00, 0x00,
            0x88, 0x77, 0x66, 0x55, 0x99, 0x00, 0x00, 0x00,
            0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01 }));
    }
    {
        // Explicit Align, and no padding when already aligned.
        FUBotCdrWriter Writer;
        Writer.Align(8);
        TestEqual(TEXT("Align at body offset 0 adds nothing"), Writer.GetBodyOffset(), 0);
        Writer.WriteU8(1);
        Writer.Align(4);
        TestEqual(TEXT("Align(4) after one byte"), Writer.GetBodyOffset(), 4);
        Writer.Align(4);
        TestEqual(TEXT("Align(4) when aligned"), Writer.GetBodyOffset(), 4);
        Writer.Align(8);
        TestEqual(TEXT("Align(8) at 4"), Writer.GetBodyOffset(), 8);
        TestEqual(TEXT("Total size includes the encapsulation header"), Writer.Num(), 12);
    }
    {
        // The reader skips padding the same way and does not care about the padding content.
        const TArray<uint8> Bytes = WithHeader({
            0x11, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x77,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF0, 0x3F });
        FUBotCdrReader Reader(Bytes);
        uint8 U8 = 0;
        double F64 = 0.0;
        TestTrue(TEXT("Read uint8"), Reader.ReadU8(U8) && U8 == 0x11);
        TestTrue(TEXT("Read float64 across non-zero padding"), Reader.ReadF64(F64) && F64 == 1.0);
        TestEqual(TEXT("Reader body offset"), Reader.GetBodyOffset(), 16);
    }
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotROSCdrStringsTest, "UBotROS.Cdr.Strings", UBotROSCdrTests::TestFlags)

bool FUBotROSCdrStringsTest::RunTest(const FString& Parameters)
{
    using namespace UBotROSCdrTests;

    const FString NonAscii = TEXT("ü中");

    FUBotCdrWriter Writer;
    Writer.WriteString(FString());
    Writer.WriteString(TEXT("abc"));
    Writer.WriteString(NonAscii);
    Writer.WriteU8(0x5A);
    const TArray<uint8> Encoded = Writer.Finish();
    ExpectBytes(*this, TEXT("Encoded strings"), Encoded, WithHeader({
        0x01, 0x00, 0x00, 0x00, 0x00,                   // "" = length 1 + NUL
        0x00, 0x00, 0x00,                               // pad to 8
        0x04, 0x00, 0x00, 0x00, 0x61, 0x62, 0x63, 0x00, // "abc"
        0x06, 0x00, 0x00, 0x00,                         // 5 UTF-8 bytes + NUL
        0xC3, 0xBC, 0xE4, 0xB8, 0xAD, 0x00,
        0x5A }));

    FUBotCdrReader Reader(Encoded);
    FString Empty = TEXT("not empty");
    FString Abc;
    FString Decoded;
    uint8 Tail = 0;
    TestTrue(TEXT("Read empty string"), Reader.ReadString(Empty) && Empty.IsEmpty());
    TestTrue(TEXT("Read ASCII string"), Reader.ReadString(Abc) && Abc == TEXT("abc"));
    TestTrue(TEXT("Read UTF-8 string"), Reader.ReadString(Decoded) && Decoded == NonAscii);
    TestTrue(TEXT("Read byte after the strings"), Reader.ReadU8(Tail) && Tail == 0x5A);

    // Tolerated encodings: length 0 for "" and a missing terminator.
    const TArray<uint8> Lenient = WithHeader({ 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x68, 0x69 });
    FUBotCdrReader LenientReader(Lenient);
    FString ZeroLength = TEXT("x");
    FString Unterminated;
    TestTrue(TEXT("Length 0 reads as empty"), LenientReader.ReadString(ZeroLength) && ZeroLength.IsEmpty());
    TestTrue(TEXT("Unterminated string is accepted"), LenientReader.ReadString(Unterminated) && Unterminated == TEXT("hi"));

    // String sequences.
    FUBotCdrWriter SequenceWriter;
    SequenceWriter.WriteStringSequence({ FString(TEXT("a")), FString(TEXT("bc")) });
    const TArray<uint8> SequenceEncoded = SequenceWriter.Finish();
    ExpectBytes(*this, TEXT("Encoded string sequence"), SequenceEncoded, WithHeader({
        0x02, 0x00, 0x00, 0x00,
        0x02, 0x00, 0x00, 0x00, 0x61, 0x00,
        0x00, 0x00,
        0x03, 0x00, 0x00, 0x00, 0x62, 0x63, 0x00 }));
    FUBotCdrReader SequenceReader(SequenceEncoded);
    TArray<FString> Strings;
    TestTrue(TEXT("Read string sequence"), SequenceReader.ReadStringSequence(Strings));
    TestTrue(TEXT("String sequence content"), Strings.Num() == 2 && Strings[0] == TEXT("a") && Strings[1] == TEXT("bc"));
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotROSCdrSequencesTest, "UBotROS.Cdr.Sequences", UBotROSCdrTests::TestFlags)

bool FUBotROSCdrSequencesTest::RunTest(const FString& Parameters)
{
    using namespace UBotROSCdrTests;

    const TArray<uint8> Bytes = { 1, 2, 3 };
    const TArray<float> Floats = { 1.5f };
    const TArray<double> NoDoubles;
    const TArray<double> Doubles = { 0.25 };

    FUBotCdrWriter Writer;
    Writer.WriteU8Sequence(Bytes);
    Writer.WriteF32Sequence(Floats);
    Writer.WriteF64Sequence(NoDoubles);
    Writer.WriteU8(7);
    Writer.WriteF64Sequence(Doubles);
    const TArray<uint8> Encoded = Writer.Finish();
    ExpectBytes(*this, TEXT("Encoded sequences"), Encoded, WithHeader({
        0x03, 0x00, 0x00, 0x00, 0x01, 0x02, 0x03,       // uint8[3]
        0x00,                                           // pad to 8
        0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x3F, // float32[1]
        0x00, 0x00, 0x00, 0x00,                         // float64[0]: no element alignment
        0x07,                                           // uint8 at body 20
        0x00, 0x00, 0x00,                               // pad to 24
        0x01, 0x00, 0x00, 0x00,                         // float64[1] count
        0x00, 0x00, 0x00, 0x00,                         // pad to 32
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xD0, 0x3F }));

    FUBotCdrReader Reader(Encoded);
    TArray<uint8> ReadBytes;
    TArray<float> ReadFloats;
    TArray<double> ReadNoDoubles = { 9.0 };
    TArray<double> ReadDoubles;
    uint8 Marker = 0;
    TestTrue(TEXT("Read uint8[]"), Reader.ReadU8Sequence(ReadBytes) && ReadBytes == Bytes);
    TestTrue(TEXT("Read float32[]"), Reader.ReadF32Sequence(ReadFloats) && ReadFloats == Floats);
    TestTrue(TEXT("Read empty float64[]"), Reader.ReadF64Sequence(ReadNoDoubles) && ReadNoDoubles.IsEmpty());
    TestTrue(TEXT("Read uint8 after the empty sequence"), Reader.ReadU8(Marker) && Marker == 7);
    TestTrue(TEXT("Read float64[]"), Reader.ReadF64Sequence(ReadDoubles) && ReadDoubles == Doubles);
    TestEqual(TEXT("Everything consumed"), Reader.GetRemaining(), 0);

    // Raw bytes: no alignment, no length.
    FUBotCdrWriter RawWriter;
    RawWriter.WriteU8(1);
    RawWriter.WriteBytes(TArray<uint8>({ 0xA1, 0xA2, 0xA3 }));
    ExpectBytes(*this, TEXT("Raw bytes"), RawWriter.Finish(), WithHeader({ 0x01, 0xA1, 0xA2, 0xA3 }));
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotROSCdrArraysTest, "UBotROS.Cdr.Arrays", UBotROSCdrTests::TestFlags)

bool FUBotROSCdrArraysTest::RunTest(const FString& Parameters)
{
    using namespace UBotROSCdrTests;

    TStaticArray<double, 3> Doubles(InPlace, 0.0);
    Doubles[0] = 1.0;
    Doubles[1] = -2.0;
    Doubles[2] = 0.5;
    const float Floats[2] = { 0.5f, -0.5f };

    FUBotCdrWriter Writer;
    Writer.WriteU32(3);
    Writer.WriteF64Array(Doubles);
    Writer.WriteF32Array(MakeArrayView(Floats));
    const TArray<uint8> Encoded = Writer.Finish();
    ExpectBytes(*this, TEXT("Encoded fixed arrays"), Encoded, WithHeader({
        0x03, 0x00, 0x00, 0x00,                         // uint32
        0x00, 0x00, 0x00, 0x00,                         // pad to 8; no length prefix
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF0, 0x3F,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, 0x3F,
        0x00, 0x00, 0x00, 0x3F, 0x00, 0x00, 0x00, 0xBF }));

    FUBotCdrReader Reader(Encoded);
    uint32 Count = 0;
    TStaticArray<double, 3> ReadDoubles(InPlace, 0.0);
    float ReadFloats[2] = { 0.0f, 0.0f };
    TestTrue(TEXT("Read uint32"), Reader.ReadU32(Count) && Count == 3);
    TestTrue(TEXT("Read float64[3]"), Reader.ReadF64Array(ReadDoubles) && ReadDoubles == Doubles);
    TestTrue(TEXT("Read float32[2]"), Reader.ReadF32Array(MakeArrayView(ReadFloats)) && ReadFloats[0] == 0.5f && ReadFloats[1] == -0.5f);
    TestEqual(TEXT("Everything consumed"), Reader.GetRemaining(), 0);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotROSCdrReaderBoundsTest, "UBotROS.Cdr.ReaderBounds", UBotROSCdrTests::TestFlags)

bool FUBotROSCdrReaderBoundsTest::RunTest(const FString& Parameters)
{
    using namespace UBotROSCdrTests;

    // Invalid or missing encapsulation header.
    {
        const TArray<uint8> Empty;
        FUBotCdrReader Reader(Empty);
        uint32 Value = 7;
        TestFalse(TEXT("Empty buffer is rejected"), Reader.IsOk());
        TestFalse(TEXT("Read from a rejected buffer fails"), Reader.ReadU32(Value));
        TestEqual(TEXT("Failed read zeroes its output"), static_cast<int64>(Value), static_cast<int64>(0));
        TestEqual(TEXT("No bytes remain in a failed reader"), Reader.GetRemaining(), 0);
    }
    {
        const TArray<uint8> Short = { 0x00, 0x01, 0x00 };
        TestFalse(TEXT("3-byte buffer is rejected"), FUBotCdrReader(Short).IsOk());
        const TArray<uint8> PlCdr = { 0x00, 0x03, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00 };
        TestFalse(TEXT("PL_CDR_LE is rejected"), FUBotCdrReader(PlCdr).IsOk());
        const TArray<uint8> Garbage = { 0x55, 0x42, 0x52, 0x42, 0x01, 0x00, 0x00, 0x00 };
        TestFalse(TEXT("Unknown representation is rejected"), FUBotCdrReader(Garbage).IsOk());
    }

    // Truncated primitives, including truncation inside alignment padding.
    {
        const TArray<uint8> HeaderOnly = WithHeader({});
        FUBotCdrReader Reader(HeaderOnly);
        uint8 Value = 9;
        TestTrue(TEXT("Header-only buffer is valid"), Reader.IsOk());
        TestFalse(TEXT("Reading past the end fails"), Reader.ReadU8(Value));
        TestEqual(TEXT("Failed uint8 is zeroed"), static_cast<int32>(Value), 0);
        TestFalse(TEXT("Reader is failed afterwards"), Reader.IsOk());
    }
    {
        const TArray<uint8> Bytes = WithHeader({ 0x01, 0x02, 0x03 });
        FUBotCdrReader Reader(Bytes);
        uint32 Value = 1;
        TestFalse(TEXT("uint32 from 3 bytes fails"), Reader.ReadU32(Value));
        TestEqual(TEXT("Failed uint32 is zeroed"), static_cast<int64>(Value), static_cast<int64>(0));
    }
    {
        const TArray<uint8> Bytes = WithHeader({ 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF0 });
        FUBotCdrReader Reader(Bytes);
        uint8 Byte = 0;
        double Value = 1.0;
        TestTrue(TEXT("uint8 before a truncated float64"), Reader.ReadU8(Byte));
        TestFalse(TEXT("float64 missing its last byte fails"), Reader.ReadF64(Value));
        TestEqual(TEXT("Failed float64 is zeroed"), Value, 0.0, 0.0);
    }
    {
        const TArray<uint8> Bytes = WithHeader({ 0x01, 0x00 });
        FUBotCdrReader Reader(Bytes);
        uint8 Byte = 0;
        uint16 Short = 0;
        uint32 Value = 0;
        TestTrue(TEXT("uint8"), Reader.ReadU8(Byte));
        TestFalse(TEXT("Alignment padding beyond the end fails"), Reader.ReadU32(Value));
        TestFalse(TEXT("Errors are sticky"), Reader.ReadU16(Short));
    }

    // Lengths and counts larger than the data.
    {
        const TArray<uint8> Bytes = WithHeader({ 0x64, 0x00, 0x00, 0x00, 0x61, 0x62, 0x63 });
        FUBotCdrReader Reader(Bytes);
        FString Value = TEXT("old");
        TestFalse(TEXT("String longer than the buffer fails"), Reader.ReadString(Value));
        TestTrue(TEXT("Failed string is empty"), Value.IsEmpty());
    }
    {
        const TArray<uint8> Bytes = WithHeader({ 0xFF, 0xFF, 0xFF, 0xFF, 0x61 });
        FUBotCdrReader Reader(Bytes);
        FString Value;
        TestFalse(TEXT("String length 0xFFFFFFFF fails"), Reader.ReadString(Value));
    }
    {
        const TArray<uint8> Bytes = WithHeader({ 0xFF, 0xFF, 0xFF, 0x7F, 0x00, 0x00, 0x80, 0x3F });
        FUBotCdrReader Reader(Bytes);
        TArray<float> Values = { 1.0f };
        TestFalse(TEXT("Huge float32[] count fails before allocating"), Reader.ReadF32Sequence(Values));
        TestTrue(TEXT("Failed sequence is empty"), Values.IsEmpty());
    }
    {
        const TArray<uint8> Bytes = WithHeader({ 0xFF, 0xFF, 0xFF, 0xFF });
        FUBotCdrReader Reader(Bytes);
        int32 Count = 5;
        TestFalse(TEXT("Count above MAX_int32 fails"), Reader.ReadSequenceLength(Count));
        TestEqual(TEXT("Failed count is zero"), Count, 0);
    }
    {
        // Two float32 announced, one present.
        const TArray<uint8> Bytes = WithHeader({ 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00 });
        FUBotCdrReader Reader(Bytes);
        TArray<float> Values;
        TestFalse(TEXT("Truncated float32[] fails"), Reader.ReadF32Sequence(Values));
        TestTrue(TEXT("Truncated sequence is empty"), Values.IsEmpty());
    }
    {
        // A TFMessage announcing far more transforms than the data can hold.
        const TArray<uint8> Bytes = WithHeader({ 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00 });
        FUBotRosTfMessage Message;
        TestFalse(TEXT("Corrupt TFMessage count fails"), UBot::Ros::Decode(Bytes, Message));
        TestTrue(TEXT("No transforms allocated"), Message.transforms.IsEmpty());
    }
    {
        const TArray<uint8> Bytes = WithHeader({ 0x02 });
        FUBotCdrReader Reader(Bytes);
        bool bValue = true;
        TestFalse(TEXT("bool other than 0/1 fails"), Reader.ReadBool(bValue));
        TestFalse(TEXT("Failed bool is false"), bValue);
    }
    {
        const TArray<uint8> Bytes = WithHeader({ 0x01, 0x02 });
        FUBotCdrReader Reader(Bytes);
        uint8 Raw[3] = { 9, 9, 9 };
        TestFalse(TEXT("Raw read beyond the end fails"), Reader.ReadBytes(MakeArrayView(Raw)));
        TestTrue(TEXT("Failed raw read is zeroed"), Raw[0] == 0 && Raw[1] == 0 && Raw[2] == 0);
    }

    // Big-endian data (encapsulation 00 00) is byte-swapped.
    {
        const TArray<uint8> Bytes = { 0x00, 0x00, 0x00, 0x00, 0x12, 0x34, 0x00, 0x00, 0xDE, 0xAD, 0xBE, 0xEF,
            0x3F, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
        FUBotCdrReader Reader(Bytes);
        uint16 U16 = 0;
        uint32 U32 = 0;
        double F64 = 0.0;
        TestTrue(TEXT("CDR_BE is accepted"), Reader.IsOk() && Reader.IsBigEndian());
        TestTrue(TEXT("Big-endian uint16"), Reader.ReadU16(U16) && U16 == 0x1234);
        TestTrue(TEXT("Big-endian uint32"), Reader.ReadU32(U32) && U32 == 0xDEADBEEF);
        TestTrue(TEXT("Big-endian float64"), Reader.ReadF64(F64) && F64 == 1.0);
    }

    // Trailing bytes after a complete message are ignored (rmw may round the size up).
    {
        TArray<uint8> Bytes(MakeGolden(UBotROSGoldenData::Twist, UBotROSGoldenData::TwistLayout).Bytes);
        Bytes.Append({ 0xEE, 0xEE, 0xEE });
        FUBotRosTwist Twist;
        TestTrue(TEXT("Twist with trailing bytes decodes"), UBot::Ros::Decode(Bytes, Twist));
        ExpectSameTwist(*this, TEXT("Twist with trailing bytes"), Twist, MakeTwist());
    }
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotROSCdrTwistRoundTripTest, "UBotROS.Cdr.TwistRoundTrip", UBotROSCdrTests::TestFlags)

bool FUBotROSCdrTwistRoundTripTest::RunTest(const FString& Parameters)
{
    using namespace UBotROSCdrTests;

    // The golden Twist as published by rclpy decodes to the expected values.
    {
        FUBotRosTwist Twist;
        const bool bDecoded = UBot::Ros::Decode(MakeGolden(UBotROSGoldenData::Twist, UBotROSGoldenData::TwistLayout).Bytes, Twist);
        TestTrue(TEXT("Golden Twist decodes"), bDecoded);
        ExpectSameTwist(*this, TEXT("Golden Twist"), Twist, MakeTwist());
    }

    // Encode -> Decode is lossless, including special values.
    FUBotRosTwist Special;
    Special.linear = MakeVector3(-0.0, std::numeric_limits<double>::max(), -std::numeric_limits<double>::infinity());
    Special.angular = MakeVector3(std::numeric_limits<double>::denorm_min(), -1.0e-300, 123456.789);
    for (const FUBotRosTwist& Original : { MakeTwist(), FUBotRosTwist(), Special })
    {
        const TArray<uint8> Encoded = UBot::Ros::Encode(Original);
        TestEqual(TEXT("Twist is 52 bytes"), Encoded.Num(), 52);
        FUBotRosTwist Decoded;
        Decoded.linear = MakeVector3(9.0, 9.0, 9.0);
        TestTrue(TEXT("Twist decodes"), UBot::Ros::Decode(Encoded, Decoded));
        ExpectSameTwist(*this, TEXT("Twist round trip"), Decoded, Original);
    }

    // Decoding stops cleanly on a truncated Twist.
    const TArray<uint8> Encoded = UBot::Ros::Encode(MakeTwist());
    FUBotRosTwist Truncated;
    TestFalse(TEXT("Twist missing one byte fails"), UBot::Ros::Decode(TConstArrayView<uint8>(Encoded).Left(Encoded.Num() - 1), Truncated));
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotROSCdrHelpersTest, "UBotROS.Cdr.Helpers", UBotROSCdrTests::TestFlags)

bool FUBotROSCdrHelpersTest::RunTest(const FString& Parameters)
{
    using namespace UBotROSCdrTests;

    const FUBotRosTime Stamp = FUBotRosTime::FromSeconds(12.5);
    TestEqual(TEXT("FromSeconds sec"), Stamp.sec, 12);
    TestEqual(TEXT("FromSeconds nanosec"), static_cast<int64>(Stamp.nanosec), static_cast<int64>(500000000));
    TestEqual(TEXT("ToSeconds"), Stamp.ToSeconds(), 12.5, 1.e-12);
    const FUBotRosTime Negative = FUBotRosTime::FromSeconds(-3.0);
    TestTrue(TEXT("Negative seconds clamp to zero"), Negative.sec == 0 && Negative.nanosec == 0);

    FUBotRosQuaternion Identity;
    TestTrue(TEXT("Default quaternion is the identity"), Identity.x == 0.0 && Identity.y == 0.0 && Identity.z == 0.0 && Identity.w == 1.0);
    FUBotRosImu DefaultImu;
    TestTrue(TEXT("Default covariances are zero"), DefaultImu.orientation_covariance[0] == 0.0 && DefaultImu.linear_acceleration_covariance[8] == 0.0);
    const FUBotRosVector3 Copied = FUBotRosVector3::FromVector(FVector(1.0, -2.0, 3.0));
    TestTrue(TEXT("FromVector copies verbatim"), Copied.x == 1.0 && Copied.y == -2.0 && Copied.z == 3.0);
    TestTrue(TEXT("FromQuat copies verbatim"), FUBotRosQuaternion::FromQuat(FQuat(0.1, 0.2, 0.3, 0.4)).ToQuat().Equals(FQuat(0.1, 0.2, 0.3, 0.4), 0.0));

    TestTrue(TEXT("PointField datatype constants"), FUBotRosPointField::INT8 == 1 && FUBotRosPointField::UINT16 == 4
        && FUBotRosPointField::FLOAT32 == 7 && FUBotRosPointField::FLOAT64 == 8);

    TestTrue(TEXT("QoS default is SensorData"), FUBotROSQos() == FUBotROSQos::SensorData());
    TestTrue(TEXT("SensorData QoS"), FUBotROSQos::SensorData() == FUBotROSQos(EUBotROSReliability::BestEffort, EUBotROSDurability::Volatile, 5));
    TestTrue(TEXT("State QoS"), FUBotROSQos::State() == FUBotROSQos(EUBotROSReliability::Reliable, EUBotROSDurability::Volatile, 10));
    TestTrue(TEXT("Latched QoS"), FUBotROSQos::Latched() == FUBotROSQos(EUBotROSReliability::Reliable, EUBotROSDurability::TransientLocal, 1));
    TestTrue(TEXT("Tf QoS"), FUBotROSQos::Tf() == FUBotROSQos(EUBotROSReliability::Reliable, EUBotROSDurability::Volatile, 100));
    return true;
}

// -------------------------------------------------------------------------------------------------
// Golden bytes from rclpy.serialization.serialize_message (UBotROSGoldenData.h)

#define UBOTROS_GOLDEN_TEST(Name, TypeNameString, MakeMessage) \
    IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotROSCdrGolden##Name##Test, "UBotROS.Cdr.Golden." #Name, UBotROSCdrTests::TestFlags) \
    bool FUBotROSCdrGolden##Name##Test::RunTest(const FString& Parameters) \
    { \
        using namespace UBotROSCdrTests; \
        CheckGolden(*this, UBotROSCdrTests::MakeMessage, MakeGolden(UBotROSGoldenData::Name, UBotROSGoldenData::Name##Layout), TEXT(TypeNameString)); \
        return true; \
    }

UBOTROS_GOLDEN_TEST(Time, "builtin_interfaces/msg/Time", MakeTime(1700000123, 456789012))
UBOTROS_GOLDEN_TEST(Header, "std_msgs/msg/Header", MakeHeader(12, 500000000, TEXT("map")))
UBOTROS_GOLDEN_TEST(HeaderEmptyFrame, "std_msgs/msg/Header", MakeHeader(0, 0, FString()))
UBOTROS_GOLDEN_TEST(PoseStamped, "geometry_msgs/msg/PoseStamped", MakePoseStamped())
UBOTROS_GOLDEN_TEST(TransformStamped, "geometry_msgs/msg/TransformStamped", MakeSingleTransformStamped())
UBOTROS_GOLDEN_TEST(Imu, "sensor_msgs/msg/Imu", MakeImu())
UBOTROS_GOLDEN_TEST(Odometry, "nav_msgs/msg/Odometry", MakeOdometry())
UBOTROS_GOLDEN_TEST(LaserScan, "sensor_msgs/msg/LaserScan", MakeLaserScan())
UBOTROS_GOLDEN_TEST(PointField, "sensor_msgs/msg/PointField", MakeSinglePointField())
UBOTROS_GOLDEN_TEST(PointCloud2, "sensor_msgs/msg/PointCloud2", MakePointCloud2())
UBOTROS_GOLDEN_TEST(Image, "sensor_msgs/msg/Image", MakeImage())
UBOTROS_GOLDEN_TEST(RegionOfInterest, "sensor_msgs/msg/RegionOfInterest", MakeRegionOfInterest(1, 2, 3, 4, true))
UBOTROS_GOLDEN_TEST(CameraInfo, "sensor_msgs/msg/CameraInfo", MakeCameraInfo())
UBOTROS_GOLDEN_TEST(TfMessage, "tf2_msgs/msg/TFMessage", MakeTfMessage())
UBOTROS_GOLDEN_TEST(Clock, "rosgraph_msgs/msg/Clock", MakeClock())
UBOTROS_GOLDEN_TEST(Twist, "geometry_msgs/msg/Twist", MakeTwist())
UBOTROS_GOLDEN_TEST(AttitudeTarget, "mavros_msgs/msg/AttitudeTarget", MakeAttitudeTarget())
UBOTROS_GOLDEN_TEST(Actuators, "actuator_msgs/msg/Actuators", MakeActuators())

#undef UBOTROS_GOLDEN_TEST

#endif // WITH_DEV_AUTOMATION_TESTS
