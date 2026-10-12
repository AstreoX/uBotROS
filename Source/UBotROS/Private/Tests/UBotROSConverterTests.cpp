#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"

#include <limits>

#if WITH_DEV_AUTOMATION_TESTS

#include "Components/BoxComponent.h"
#include "Engine/CollisionProfile.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "UBot2DLidarFrame.h"
#include "UBot2DLidarSensorComponent.h"
#include "UBot3DLidarSensorComponent.h"
#include "UBotActuatorInterface.h"
#include "UBotImageFrame.h"
#include "UBotPointCloudFrame.h"
#include "UBotROSConverters.h"
#include "UBotROSMessageTypes.h"
#include "UBotRosConventions.h"
#include "UBotSensorTypes.h"
#include "UBotTypes.h"
#include "UBotUnits.h"

// Befriended by UUBotImageFrame: fills CPU pixel data directly so the converters can be tested without a
// render target or GPU readback. UBotSensor's camera tests define the same struct; this definition must
// stay token-identical to theirs so a monolithic build that links both test modules keeps one definition.
struct FUBotImageFrameTestAccess
{
    static TArray<uint8>& PixelBytes(UUBotImageFrame& Frame)
    {
        return Frame.PixelBytes;
    }

    static TArray<float>& DepthValuesMeters(UUBotImageFrame& Frame)
    {
        return Frame.DepthValuesMeters;
    }
};

namespace UBotROSConverterTests
{
    constexpr double Tolerance = 1.e-9;
    constexpr EAutomationTestFlags TestFlags = EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter;

    // Transient game world with a physics scene, so line traces work right after components register.
    // Same setup as the UBotSensor LiDAR tests. Destroyed when the scope ends.
    class FScopedTestWorld
    {
    public:
        UE_NONCOPYABLE(FScopedTestWorld);

        FScopedTestWorld()
            : World(UWorld::CreateWorld(EWorldType::Game, false))
        {
        }

        ~FScopedTestWorld()
        {
            if (World)
            {
                World->DestroyWorld(false);
            }
        }

        UWorld* Get() const
        {
            return World;
        }

        // Blocking axis-aligned box centered on CenterCentimeters.
        AActor* SpawnBox(const FVector& CenterCentimeters, const FVector& HalfExtentCentimeters) const
        {
            AActor* Target = World ? World->SpawnActor<AActor>() : nullptr;
            if (!Target)
            {
                return nullptr;
            }
            UBoxComponent* Box = NewObject<UBoxComponent>(Target, TEXT("Box"));
            Box->SetBoxExtent(HalfExtentCentimeters, false);
            Box->SetCollisionProfileName(UCollisionProfile::BlockAll_ProfileName);
            Target->SetRootComponent(Box);
            Box->SetWorldLocation(CenterCentimeters);
            Box->RegisterComponent();
            return Target;
        }

        // LiDAR as the root of its own carrier actor at the world origin, facing +X.
        template <typename TLidarComponent>
        TLidarComponent* SpawnLidar() const
        {
            AActor* Carrier = World ? World->SpawnActor<AActor>() : nullptr;
            if (!Carrier)
            {
                return nullptr;
            }
            TLidarComponent* Lidar = NewObject<TLidarComponent>(Carrier, TEXT("Lidar"));
            Carrier->SetRootComponent(Lidar);
            Lidar->RegisterComponent();
            return Lidar;
        }

    private:
        UWorld* World = nullptr;
    };

    uint16 ReadUInt16LE(const TArray<uint8>& Data, int32 Offset)
    {
        return static_cast<uint16>(Data[Offset] | (Data[Offset + 1] << 8));
    }

    uint32 ReadUInt32LE(const TArray<uint8>& Data, int32 Offset)
    {
        return static_cast<uint32>(Data[Offset])
            | (static_cast<uint32>(Data[Offset + 1]) << 8)
            | (static_cast<uint32>(Data[Offset + 2]) << 16)
            | (static_cast<uint32>(Data[Offset + 3]) << 24);
    }

    float ReadFloatLE(const TArray<uint8>& Data, int32 Offset)
    {
        const uint32 Bits = ReadUInt32LE(Data, Offset);
        float Value = 0.0f;
        FMemory::Memcpy(&Value, &Bits, sizeof(Value));
        return Value;
    }

    /** Largest component difference between A and B, treating q and -q as the same rotation. */
    double QuatError(const FQuat& A, const FQuat& B)
    {
        const double Same = FMath::Max(FMath::Max(FMath::Abs(A.X - B.X), FMath::Abs(A.Y - B.Y)), FMath::Max(FMath::Abs(A.Z - B.Z), FMath::Abs(A.W - B.W)));
        const double Opposite = FMath::Max(FMath::Max(FMath::Abs(A.X + B.X), FMath::Abs(A.Y + B.Y)), FMath::Max(FMath::Abs(A.Z + B.Z), FMath::Abs(A.W + B.W)));
        return FMath::Min(Same, Opposite);
    }

    /** Maps a point given in the transform's child frame into its parent frame (ROS meters). */
    FVector ApplyRosTransform(const FUBotRosTransform& Transform, const FVector& ChildPoint)
    {
        return Transform.rotation.ToQuat().RotateVector(ChildPoint) + Transform.translation.ToVector();
    }

    /** Yaw (rotation about +Z, counter-clockwise seen from above in ROS) of a ROS quaternion. */
    double RosYaw(const FUBotRosQuaternion& Value)
    {
        return FMath::Atan2(2.0 * (Value.w * Value.z + Value.x * Value.y), 1.0 - 2.0 * (Value.y * Value.y + Value.z * Value.z));
    }

    void TestStamp(FAutomationTestBase& Test, const FString& What, const FUBotRosTime& Stamp, int32 ExpectedSec, int64 ExpectedNanosec)
    {
        Test.TestEqual(What + TEXT(" sec"), static_cast<int32>(Stamp.sec), ExpectedSec);
        Test.TestEqual(*(What + TEXT(" nanosec")), static_cast<int64>(Stamp.nanosec), ExpectedNanosec);
    }

    /** Fixed uBot PointCloud2 layout: x, y, z, intensity float32 @ 0/4/8/12, ring uint16 @ 16, time float32 @ 18. */
    void TestPointCloudLayout(FAutomationTestBase& Test, const FUBotRosPointCloud2& Cloud, int32 ExpectedPointCount)
    {
        struct FExpectedField
        {
            const TCHAR* Name;
            int32 Offset;
            int32 Datatype;
        };
        // sensor_msgs/PointField: UINT16 = 4, FLOAT32 = 7.
        const FExpectedField ExpectedFields[] = {
            { TEXT("x"), 0, 7 },
            { TEXT("y"), 4, 7 },
            { TEXT("z"), 8, 7 },
            { TEXT("intensity"), 12, 7 },
            { TEXT("ring"), 16, 4 },
            { TEXT("time"), 18, 7 } };

        Test.TestEqual(TEXT("PointField FLOAT32 constant"), static_cast<int32>(FUBotRosPointField::FLOAT32), 7);
        Test.TestEqual(TEXT("PointField UINT16 constant"), static_cast<int32>(FUBotRosPointField::UINT16), 4);
        if (!Test.TestEqual(TEXT("Six point fields"), Cloud.fields.Num(), 6))
        {
            return;
        }
        for (int32 Index = 0; Index < 6; ++Index)
        {
            const FUBotRosPointField& Field = Cloud.fields[Index];
            const FExpectedField& Expected = ExpectedFields[Index];
            Test.TestEqual(FString::Printf(TEXT("Field %d name"), Index), *Field.name, Expected.Name);
            Test.TestEqual(FString::Printf(TEXT("Field %s offset"), Expected.Name), static_cast<int32>(Field.offset), Expected.Offset);
            Test.TestEqual(FString::Printf(TEXT("Field %s datatype"), Expected.Name), static_cast<int32>(Field.datatype), Expected.Datatype);
            Test.TestEqual(FString::Printf(TEXT("Field %s count"), Expected.Name), static_cast<int32>(Field.count), 1);
        }

        Test.TestEqual(TEXT("point_step is 22 bytes"), static_cast<int32>(Cloud.point_step), 22);
        Test.TestEqual(TEXT("Layout constant matches point_step"), static_cast<int32>(UBot::ROSConvert::PointCloudPointStep), 22);
        Test.TestEqual(TEXT("Unorganized cloud has height 1"), static_cast<int32>(Cloud.height), 1);
        Test.TestEqual(TEXT("width is the point count"), static_cast<int32>(Cloud.width), ExpectedPointCount);
        Test.TestEqual(TEXT("row_step is point_step * width"), static_cast<int32>(Cloud.row_step), 22 * ExpectedPointCount);
        Test.TestEqual(TEXT("data holds row_step bytes"), Cloud.data.Num(), 22 * ExpectedPointCount);
        Test.TestFalse(TEXT("Little endian"), static_cast<bool>(Cloud.is_bigendian));
        Test.TestTrue(TEXT("Finite cloud is dense"), static_cast<bool>(Cloud.is_dense));
    }

    /**
     * Decodes every 22-byte record (explicitly little endian) and compares it with the source frame:
     * x, z unchanged, y negated, intensity copied or 0 without intensity data, ring and time copied.
     */
    void TestPointCloudRecords(FAutomationTestBase& Test, const FUBotRosPointCloud2& Cloud, const UUBotPointCloudFrame& Frame)
    {
        const TArray<FVector>& Points = Frame.GetPointsMeters();
        const TArray<int32>& Rings = Frame.GetRingIndices();
        const TArray<float>& TimeOffsets = Frame.GetTimeOffsetsSeconds();
        const TArray<float>& Intensities = Frame.GetIntensities();
        if (Cloud.data.Num() != Points.Num() * 22 || Rings.Num() != Points.Num() || TimeOffsets.Num() != Points.Num())
        {
            Test.AddError(TEXT("Point cloud record check needs aligned frame arrays and a complete payload."));
            return;
        }

        int32 Mismatches = 0;
        for (int32 Index = 0; Index < Points.Num(); ++Index)
        {
            const int32 Offset = Index * 22;
            const float ExpectedX = static_cast<float>(Points[Index].X);
            const float ExpectedY = static_cast<float>(-Points[Index].Y);
            const float ExpectedZ = static_cast<float>(Points[Index].Z);
            const float ExpectedIntensity = Frame.HasIntensityData() ? Intensities[Index] : 0.0f;
            const int32 ExpectedRing = Rings[Index];
            const float ExpectedTime = TimeOffsets[Index];

            const float X = ReadFloatLE(Cloud.data, Offset);
            const float Y = ReadFloatLE(Cloud.data, Offset + 4);
            const float Z = ReadFloatLE(Cloud.data, Offset + 8);
            const float Intensity = ReadFloatLE(Cloud.data, Offset + 12);
            const int32 Ring = ReadUInt16LE(Cloud.data, Offset + 16);
            const float Time = ReadFloatLE(Cloud.data, Offset + 18);
            const bool bMatches = X == ExpectedX && Y == ExpectedY && Z == ExpectedZ && Intensity == ExpectedIntensity && Ring == ExpectedRing && Time == ExpectedTime;
            if (!bMatches && Mismatches++ == 0)
            {
                Test.AddError(FString::Printf(
                    TEXT("Point %d: got (%f, %f, %f) i=%f ring=%d t=%f, expected (%f, %f, %f) i=%f ring=%d t=%f"),
                    Index, X, Y, Z, Intensity, Ring, Time, ExpectedX, ExpectedY, ExpectedZ, ExpectedIntensity, ExpectedRing, ExpectedTime));
            }
        }
        Test.TestEqual(TEXT("Every record matches its frame point"), Mismatches, 0);
    }
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotROSConvertHeaderTest, "UBotROS.Convert.Header", UBotROSConverterTests::TestFlags)

bool FUBotROSConvertHeaderTest::RunTest(const FString& Parameters)
{
    using namespace UBotROSConverterTests;

    FUBotFrameHeader Header;
    Header.FrameId = TEXT("raw_sensor_frame");
    Header.TimestampSeconds = 4.25;
    const FUBotRosHeader Converted = UBot::ROSConvert::MakeHeader(Header, TEXT("ns/imu_link"));
    TestEqual(TEXT("frame_id is the caller-resolved id, not the raw frame id"), *Converted.frame_id, TEXT("ns/imu_link"));
    TestStamp(*this, TEXT("4.25 s"), Converted.stamp, 4, 250000000);

    Header.TimestampSeconds = -1.0;
    TestStamp(*this, TEXT("Negative time"), UBot::ROSConvert::MakeHeader(Header, TEXT("imu_link")).stamp, 0, 0);
    Header.TimestampSeconds = 1.0e-9;
    TestStamp(*this, TEXT("One nanosecond"), UBot::ROSConvert::MakeHeader(Header, TEXT("imu_link")).stamp, 0, 1);
    Header.TimestampSeconds = 12.5;
    TestStamp(*this, TEXT("12.5 s"), UBot::ROSConvert::MakeHeader(Header, TEXT("imu_link")).stamp, 12, 500000000);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotROSConvertAngularVelocitySignTest, "UBotROS.Convert.AngularVelocitySign", UBotROSConverterTests::TestFlags)

bool FUBotROSConvertAngularVelocitySignTest::RunTest(const FString& Parameters)
{
    using namespace UBotROSConverterTests;

    // UE +Z angular velocity is a yaw to the right. In ROS (REP-103, Z up, counter-clockwise positive)
    // turning right is a negative yaw rate. The legacy bridge sent +Z here (the bug this guards).
    FUBotImuSensorData Imu;
    Imu.AngularVelocityRadPerSecond = FVector(0.0, 0.0, 1.0);
    const FUBotRosImu ImuMessage = UBot::ROSConvert::MakeImu(Imu, TEXT("imu_link"));
    TestEqual(TEXT("IMU: UE yaw-right rate becomes negative ROS z"), ImuMessage.angular_velocity.z, -1.0, Tolerance);
    TestEqual(TEXT("IMU: no x from a pure yaw rate"), ImuMessage.angular_velocity.x, 0.0, Tolerance);
    TestEqual(TEXT("IMU: no y from a pure yaw rate"), ImuMessage.angular_velocity.y, 0.0, Tolerance);

    FUBotOdometrySensorData Odometry;
    Odometry.AngularVelocityLocalRadPerSecond = FVector(0.0, 0.0, 1.0);
    const FUBotRosOdometry OdometryMessage = UBot::ROSConvert::MakeOdometry(Odometry, TEXT("odom"), TEXT("base_link"));
    TestEqual(TEXT("Odometry: UE yaw-right rate becomes negative ROS z"), OdometryMessage.twist.twist.angular.z, -1.0, Tolerance);

    // Axial vector mapping (-x, y, -z) for all components.
    Imu.AngularVelocityRadPerSecond = FVector(0.1, 0.2, 0.3);
    Odometry.AngularVelocityLocalRadPerSecond = FVector(0.1, 0.2, 0.3);
    const FVector ExpectedRos(-0.1, 0.2, -0.3);
    TestEqual(TEXT("IMU angular velocity is mapped as an axial vector"), UBot::ROSConvert::MakeImu(Imu, TEXT("imu_link")).angular_velocity.ToVector(), ExpectedRos, 1.e-9f);
    TestEqual(TEXT("Odometry angular velocity is mapped as an axial vector"), UBot::ROSConvert::MakeOdometry(Odometry, TEXT("odom"), TEXT("base_link")).twist.twist.angular.ToVector(), ExpectedRos, 1.e-9f);

    // The sign agrees with the converted orientation: a body that yawed to the right in UE has a negative
    // ROS yaw, so a positive UE yaw rate must be a negative ROS yaw rate.
    Imu.OrientationQuaternion = FQuat(FRotator(0.0, 30.0, 0.0));
    const double RosYawRadians = RosYaw(UBot::ROSConvert::MakeImu(Imu, TEXT("imu_link")).orientation);
    TestEqual(TEXT("UE yaw +30 deg is ROS yaw -30 deg"), RosYawRadians, -UBot::Units::DegreesToRadians(30.0), 1.e-9);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotROSConvertPoseTest, "UBotROS.Convert.Pose", UBotROSConverterTests::TestFlags)

bool FUBotROSConvertPoseTest::RunTest(const FString& Parameters)
{
    using namespace UBotROSConverterTests;

    FUBotPoseSensorData Data;
    Data.Header.FrameId = TEXT("pose_sensor");
    Data.Header.TimestampSeconds = 4.5;
    Data.ReferenceFrameId = TEXT("raw_reference");
    Data.PositionMeters = FVector(10.0, 20.0, 30.0);
    Data.OrientationQuaternion = FQuat(FRotator(10.0, 20.0, 30.0));

    const FUBotRosPoseStamped Message = UBot::ROSConvert::MakePose(Data, TEXT("map"));
    TestEqual(TEXT("frame_id is the caller-resolved id"), *Message.header.frame_id, TEXT("map"));
    TestStamp(*this, TEXT("Pose stamp"), Message.header.stamp, 4, 500000000);
    TestEqual(TEXT("Position is (x, -y, z), meters unchanged"), Message.pose.position.ToVector(), FVector(10.0, -20.0, 30.0), 1.e-9f);

    const FQuat UE = Data.OrientationQuaternion.GetNormalized();
    TestTrue(TEXT("Orientation is (-x, y, -z, w)"), QuatError(Message.pose.orientation.ToQuat(), FQuat(-UE.X, UE.Y, -UE.Z, UE.W)) < Tolerance);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotROSConvertImuTest, "UBotROS.Convert.Imu", UBotROSConverterTests::TestFlags)

bool FUBotROSConvertImuTest::RunTest(const FString& Parameters)
{
    using namespace UBotROSConverterTests;

    FUBotImuSensorData Data;
    Data.Header.FrameId = TEXT("raw_imu");
    Data.Header.TimestampSeconds = 12.5;
    Data.OrientationQuaternion = FQuat(FRotator(10.0, 20.0, 30.0));
    Data.AngularVelocityRadPerSecond = FVector(0.1, 0.2, 0.3);
    Data.LinearAccelerationMetersPerSecondSquared = FVector(1.0, 2.0, 9.81);
    Data.OrientationVariance = FVector(0.01, 0.02, 0.03);
    Data.AngularVelocityVariance = FVector(0.04, 0.05, 0.06);
    Data.LinearAccelerationVariance = FVector(0.07, 0.08, 0.09);

    const FUBotRosImu Message = UBot::ROSConvert::MakeImu(Data, TEXT("robot/imu_link"));
    TestEqual(TEXT("frame_id is the caller-resolved id"), *Message.header.frame_id, TEXT("robot/imu_link"));
    TestStamp(*this, TEXT("IMU stamp"), Message.header.stamp, 12, 500000000);

    const FQuat UE = Data.OrientationQuaternion.GetNormalized();
    TestTrue(TEXT("Orientation is (-x, y, -z, w)"), QuatError(Message.orientation.ToQuat(), FQuat(-UE.X, UE.Y, -UE.Z, UE.W)) < Tolerance);
    TestEqual(TEXT("Angular velocity is (-x, y, -z)"), Message.angular_velocity.ToVector(), FVector(-0.1, 0.2, -0.3), 1.e-9f);
    TestEqual(TEXT("Linear acceleration is (x, -y, z)"), Message.linear_acceleration.ToVector(), FVector(1.0, -2.0, 9.81), 1.e-9f);

    const FVector Variances[3] = { Data.OrientationVariance, Data.AngularVelocityVariance, Data.LinearAccelerationVariance };
    const TStaticArray<double, 9>* Covariances[3] = { &Message.orientation_covariance, &Message.angular_velocity_covariance, &Message.linear_acceleration_covariance };
    const TCHAR* Names[3] = { TEXT("orientation_covariance"), TEXT("angular_velocity_covariance"), TEXT("linear_acceleration_covariance") };
    for (int32 Matrix = 0; Matrix < 3; ++Matrix)
    {
        const TStaticArray<double, 9>& Covariance = *Covariances[Matrix];
        for (uint32 Index = 0; Index < 9; ++Index)
        {
            double Expected = 0.0;
            if (Index == 0)
            {
                Expected = Variances[Matrix].X;
            }
            else if (Index == 4)
            {
                Expected = Variances[Matrix].Y;
            }
            else if (Index == 8)
            {
                Expected = Variances[Matrix].Z;
            }
            TestEqual(FString::Printf(TEXT("%s[%u]"), Names[Matrix], Index), Covariance[Index], Expected, Tolerance);
        }
    }
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotROSConvertOdometryTest, "UBotROS.Convert.Odometry", UBotROSConverterTests::TestFlags)

bool FUBotROSConvertOdometryTest::RunTest(const FString& Parameters)
{
    using namespace UBotROSConverterTests;

    FUBotOdometrySensorData Data;
    Data.Header.TimestampSeconds = 7.75;
    Data.OdometryFrameId = TEXT("raw_odom");
    Data.ChildFrameId = TEXT("raw_base");
    Data.PositionMeters = FVector(1.0, 2.0, 3.0);
    Data.OrientationQuaternion = FQuat(FRotator(-5.0, 120.0, 15.0));
    Data.LinearVelocityLocalMetersPerSecond = FVector(0.5, 0.25, -0.125);
    Data.AngularVelocityLocalRadPerSecond = FVector(0.1, 0.2, 0.3);
    Data.bHasVelocity = true;
    Data.PoseCovariance.SetNumZeroed(36);
    Data.TwistCovariance.SetNumZeroed(36);
    for (int32 Index = 0; Index < 36; ++Index)
    {
        Data.PoseCovariance[Index] = 1.0 + Index;
        Data.TwistCovariance[Index] = 100.0 + Index;
    }

    const FUBotRosOdometry Message = UBot::ROSConvert::MakeOdometry(Data, TEXT("robot/odom"), TEXT("robot/base_link"));
    TestEqual(TEXT("header.frame_id is the odometry frame argument"), *Message.header.frame_id, TEXT("robot/odom"));
    TestEqual(TEXT("child_frame_id is the child frame argument"), *Message.child_frame_id, TEXT("robot/base_link"));
    TestStamp(*this, TEXT("Odometry stamp"), Message.header.stamp, 7, 750000000);
    TestEqual(TEXT("Position is (x, -y, z)"), Message.pose.pose.position.ToVector(), FVector(1.0, -2.0, 3.0), 1.e-9f);

    const FQuat UE = Data.OrientationQuaternion.GetNormalized();
    TestTrue(TEXT("Orientation is (-x, y, -z, w)"), QuatError(Message.pose.pose.orientation.ToQuat(), FQuat(-UE.X, UE.Y, -UE.Z, UE.W)) < Tolerance);
    TestEqual(TEXT("Linear twist is (x, -y, z)"), Message.twist.twist.linear.ToVector(), FVector(0.5, -0.25, -0.125), 1.e-9f);
    TestEqual(TEXT("Angular twist is (-x, y, -z)"), Message.twist.twist.angular.ToVector(), FVector(-0.1, 0.2, -0.3), 1.e-9f);

    // Spot checks of the mirrored covariances (full rule in UBotROS.Convert.CovarianceSigns).
    TestEqual(TEXT("Pose covariance diagonal is unchanged"), Message.pose.covariance[7], 8.0, Tolerance);
    TestEqual(TEXT("Pose covariance (x, y) flips"), Message.pose.covariance[1], -2.0, Tolerance);
    TestEqual(TEXT("Pose covariance (x, z) keeps its sign"), Message.pose.covariance[2], 3.0, Tolerance);
    TestEqual(TEXT("Twist covariance (vy, wx) keeps its sign"), Message.twist.covariance[1 * 6 + 3], 109.0, Tolerance);
    TestEqual(TEXT("Twist covariance (vz, wz) flips"), Message.twist.covariance[2 * 6 + 5], -117.0, Tolerance);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotROSConvertCovarianceSignsTest, "UBotROS.Convert.CovarianceSigns", UBotROSConverterTests::TestFlags)

bool FUBotROSConvertCovarianceSignsTest::RunTest(const FString& Parameters)
{
    using namespace UBotROSConverterTests;

    TArray<double> UECovariance;
    UECovariance.SetNumZeroed(36);
    for (int32 Index = 0; Index < 36; ++Index)
    {
        UECovariance[Index] = 1.0 + Index;
    }

    TStaticArray<double, 36> RosCovariance(InPlace, 0.0);
    UBot::ROSConvert::ConvertCovariance6(UECovariance, RosCovariance);

    // Mirror S = diag(+, -, +, -, +, -): polar part (x, -y, z), axial part (-rx, ry, -rz). C' = S C S.
    const double Signs[6] = { 1.0, -1.0, 1.0, -1.0, 1.0, -1.0 };
    int32 Mismatches = 0;
    for (int32 Row = 0; Row < 6; ++Row)
    {
        for (int32 Column = 0; Column < 6; ++Column)
        {
            const uint32 Index = static_cast<uint32>(Row * 6 + Column);
            const double Expected = Signs[Row] * Signs[Column] * UECovariance[Index];
            if (!FMath::IsNearlyEqual(RosCovariance[Index], Expected, Tolerance))
            {
                ++Mismatches;
                AddError(FString::Printf(TEXT("Covariance (%d, %d): got %f, expected %f"), Row, Column, RosCovariance[Index], Expected));
            }
        }
    }
    TestEqual(TEXT("Every entry follows S C S"), Mismatches, 0);

    // Hand-checked entries.
    TestEqual(TEXT("(x, x) unchanged"), RosCovariance[0], 1.0, Tolerance);
    TestEqual(TEXT("(x, y) flips"), RosCovariance[1], -2.0, Tolerance);
    TestEqual(TEXT("(x, rx) flips"), RosCovariance[3], -4.0, Tolerance);
    TestEqual(TEXT("(y, rx) keeps its sign"), RosCovariance[9], 10.0, Tolerance);
    TestEqual(TEXT("(ry, rz) flips"), RosCovariance[4 * 6 + 5], -30.0, Tolerance);
    TestEqual(TEXT("(rz, rz) unchanged"), RosCovariance[35], 36.0, Tolerance);

    // Anything but 36 values cannot be a 6x6 covariance: the output is zeroed, never left stale.
    TStaticArray<double, 36> Stale(InPlace, 42.0);
    UBot::ROSConvert::ConvertCovariance6(TArray<double>{ 1.0, 2.0, 3.0 }, Stale);
    bool bAllZero = true;
    for (uint32 Index = 0; Index < 36; ++Index)
    {
        bAllZero = bAllZero && Stale[Index] == 0.0;
    }
    TestTrue(TEXT("Wrong-size input yields zeros"), bAllZero);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotROSConvertLaserScanMirrorTest, "UBotROS.Convert.LaserScanMirror", UBotROSConverterTests::TestFlags)

bool FUBotROSConvertLaserScanMirrorTest::RunTest(const FString& Parameters)
{
    using namespace UBotROSConverterTests;

    FScopedTestWorld TestWorld;
    if (!TestNotNull(TEXT("Test world"), TestWorld.Get()))
    {
        return false;
    }

    // Seven samples from UE -90 to +90 degrees, 30 degrees apart (UE angles grow to the right). A small box
    // 5 m away on the UE +30 degree ray (UE index 4) is the only return; the -30 degree ray misses.
    const double HitAngleRadians = UBot::Units::DegreesToRadians(30.0);
    const FVector BoxCenter(500.0 * FMath::Cos(HitAngleRadians), 500.0 * FMath::Sin(HitAngleRadians), 0.0);
    TestNotNull(TEXT("Target box"), TestWorld.SpawnBox(BoxCenter, FVector(25.0, 25.0, 50.0)));
    UUBot2DLidarSensorComponent* Lidar = TestWorld.SpawnLidar<UUBot2DLidarSensorComponent>();
    if (!TestNotNull(TEXT("2D LiDAR"), Lidar))
    {
        return false;
    }
    Lidar->StartAngleDegrees = -90.0f;
    Lidar->EndAngleDegrees = 90.0f;
    Lidar->HorizontalSamples = 7;
    Lidar->MinimumRangeMeters = 0.05f;
    Lidar->MaximumRangeMeters = 30.0f;

    TestTrue(TEXT("Scan succeeds"), Lidar->Sample2DLidar());
    const UUBot2DLidarFrame* Frame = Lidar->GetLatest2DLidarFrame();
    if (!TestNotNull(TEXT("2D LiDAR frame"), Frame) || !TestEqual(TEXT("Seven UE samples"), Frame->GetSampleCount(), 7))
    {
        return false;
    }

    constexpr int32 UEHitIndex = 4;
    float UEHitRange = 0.0f;
    bool bUEHit = false;
    TestTrue(TEXT("Precondition: UE +30 degree sample hits the box"), Frame->GetRangeByIndex(UEHitIndex, UEHitRange, bUEHit) && bUEHit);
    for (int32 Index = 0; Index < 7; ++Index)
    {
        float Range = 0.0f;
        bool bHit = false;
        Frame->GetRangeByIndex(Index, Range, bHit);
        if (Index != UEHitIndex)
        {
            TestFalse(FString::Printf(TEXT("Precondition: UE sample %d misses"), Index), bHit);
        }
    }

    const FUBotRosLaserScan Scan = UBot::ROSConvert::MakeLaserScan(*Frame, TEXT("robot/lidar_2d"));
    const FUBotRosTime ExpectedStamp = FUBotRosTime::FromSeconds(Frame->Header.TimestampSeconds);
    TestEqual(TEXT("frame_id is the caller-resolved id"), *Scan.header.frame_id, TEXT("robot/lidar_2d"));
    TestStamp(*this, TEXT("Scan stamp"), Scan.header.stamp, ExpectedStamp.sec, ExpectedStamp.nanosec);

    // Limits: angle_min = -(UE max) = -90 deg, angle_max = -(UE min) = +90 deg, positive 30 deg increment.
    TestEqual(TEXT("angle_min mirrors the UE maximum"), Scan.angle_min, -UE_HALF_PI, 1.e-6f);
    TestEqual(TEXT("angle_max mirrors the UE minimum"), Scan.angle_max, UE_HALF_PI, 1.e-6f);
    TestEqual(TEXT("angle_increment stays positive"), Scan.angle_increment, UE_PI / 6.0f, 1.e-6f);
    TestEqual(TEXT("range_min is copied"), Scan.range_min, 0.05f, 1.e-6f);
    TestEqual(TEXT("range_max is copied"), Scan.range_max, 30.0f, 1.e-6f);
    TestEqual(TEXT("scan_time is the scan period"), Scan.scan_time, Frame->ScanPeriodSeconds, 1.e-6f);
    TestEqual(TEXT("time_increment spreads the period over the samples"), Scan.time_increment, Frame->ScanPeriodSeconds / 6.0f, 1.e-6f);
    if (!TestEqual(TEXT("One range per sample"), Scan.ranges.Num(), 7) || !TestEqual(TEXT("One intensity per sample"), Scan.intensities.Num(), 7))
    {
        return false;
    }

    // The UE +30 degree hit appears at ROS index 7 - 1 - 4 = 2, whose ROS angle is -90 + 2 * 30 = -30 deg.
    constexpr int32 RosHitIndex = 2;
    const double RosHitAngle = Scan.angle_min + RosHitIndex * Scan.angle_increment;
    TestEqual(TEXT("ROS angle of the hit index is -30 deg"), RosHitAngle, -HitAngleRadians, 1.e-6);
    TestEqual(TEXT("Hit range is copied"), Scan.ranges[RosHitIndex], UEHitRange, 0.0f);
    TestTrue(TEXT("Hit range is a real return"), UEHitRange > 1.0f && UEHitRange < 10.0f);
    TestTrue(TEXT("Hit intensity is a surface return"), Scan.intensities[RosHitIndex] > 0.0f);
    TestEqual(TEXT("Hit intensity is the UE sample's"), Scan.intensities[RosHitIndex], Frame->GetIntensities()[UEHitIndex], 0.0f);
    for (int32 Index = 0; Index < 7; ++Index)
    {
        if (Index == RosHitIndex)
        {
            continue;
        }
        const float Range = Scan.ranges[Index];
        TestTrue(FString::Printf(TEXT("ROS sample %d without a hit is +inf (got %f)"), Index, Range), !FMath::IsFinite(Range) && Range > 0.0f);
        TestEqual(FString::Printf(TEXT("ROS sample %d without a hit has zero intensity"), Index), Scan.intensities[Index], 0.0f, 0.0f);
    }

    // Geometry: the ROS polar point of the hit equals the mirrored UE point.
    const FVector UEPoint = FVector(FMath::Cos(HitAngleRadians), FMath::Sin(HitAngleRadians), 0.0) * UEHitRange;
    const FVector RosPoint = FVector(FMath::Cos(RosHitAngle), FMath::Sin(RosHitAngle), 0.0) * Scan.ranges[RosHitIndex];
    TestEqual(TEXT("ROS hit point is the UE hit point with y negated"), RosPoint, FVector(UEPoint.X, -UEPoint.Y, UEPoint.Z), 1.e-4f);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotROSConvertLaserScanLimitsTest, "UBotROS.Convert.LaserScanLimits", UBotROSConverterTests::TestFlags)

bool FUBotROSConvertLaserScanLimitsTest::RunTest(const FString& Parameters)
{
    using namespace UBotROSConverterTests;

    // An asymmetric field of view shows which limit goes where; an empty frame invents no samples.
    UUBot2DLidarFrame* Frame = NewObject<UUBot2DLidarFrame>();
    Frame->Header.TimestampSeconds = 4.25;
    Frame->AngleMinRadians = -1.0f;
    Frame->AngleMaxRadians = 2.0f;
    Frame->AngleIncrementRadians = 0.5f;
    Frame->RangeMinMeters = 0.1f;
    Frame->RangeMaxMeters = 50.0f;
    Frame->ScanPeriodSeconds = 0.1f;

    const FUBotRosLaserScan Scan = UBot::ROSConvert::MakeLaserScan(*Frame, TEXT("laser"));
    TestEqual(TEXT("angle_min = -(UE AngleMax)"), Scan.angle_min, -2.0f, 0.0f);
    TestEqual(TEXT("angle_max = -(UE AngleMin)"), Scan.angle_max, 1.0f, 0.0f);
    TestEqual(TEXT("angle_increment is unchanged"), Scan.angle_increment, 0.5f, 0.0f);
    TestEqual(TEXT("range_min is copied"), Scan.range_min, 0.1f, 0.0f);
    TestEqual(TEXT("range_max is copied"), Scan.range_max, 50.0f, 0.0f);
    TestEqual(TEXT("scan_time is the scan period"), Scan.scan_time, 0.1f, 0.0f);
    TestEqual(TEXT("No samples, no time increment"), Scan.time_increment, 0.0f, 0.0f);
    TestEqual(TEXT("Empty frame has no ranges"), Scan.ranges.Num(), 0);
    TestEqual(TEXT("Empty frame invents no intensities"), Scan.intensities.Num(), 0);
    TestStamp(*this, TEXT("Scan stamp"), Scan.header.stamp, 4, 250000000);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotROSConvertPointCloud2LayoutTest, "UBotROS.Convert.PointCloud2Layout", UBotROSConverterTests::TestFlags)

bool FUBotROSConvertPointCloud2LayoutTest::RunTest(const FString& Parameters)
{
    using namespace UBotROSConverterTests;

    // Depth camera cloud: known points, no intensity data. 3x2 depth image, fx = fy = 2, cx = 1, cy = 0.5.
    UUBotImageFrame* Image = NewObject<UUBotImageFrame>();
    Image->Width = 3;
    Image->Height = 2;
    Image->Encoding = EUBotImageEncoding::Depth32FC1;
    Image->Fx = 2.0f;
    Image->Fy = 2.0f;
    Image->Cx = 1.0f;
    Image->Cy = 0.5f;
    Image->Header.TimestampSeconds = 3.5;
    FUBotImageFrameTestAccess::DepthValuesMeters(*Image) = { 1.0f, 0.0f, 2.0f, -1.0f, 3.0f, 4.0f };

    FString ErrorMessage;
    const UUBotPointCloudFrame* Frame = Image->GeneratePointCloud(ErrorMessage);
    if (!TestNotNull(TEXT("Depth point cloud"), Frame))
    {
        return false;
    }
    TestFalse(TEXT("Precondition: depth clouds carry no intensity"), Frame->HasIntensityData());
    if (!TestEqual(TEXT("Precondition: four valid depth pixels"), Frame->GetPointCount(), 4))
    {
        return false;
    }

    const FUBotRosPointCloud2 Cloud = UBot::ROSConvert::MakePointCloud2(*Frame, TEXT("robot/depth_camera_optical"));
    TestEqual(TEXT("frame_id is the caller-resolved id"), *Cloud.header.frame_id, TEXT("robot/depth_camera_optical"));
    TestStamp(*this, TEXT("Cloud stamp"), Cloud.header.stamp, 3, 500000000);
    TestPointCloudLayout(*this, Cloud, 4);
    if (Cloud.data.Num() != 4 * 22)
    {
        return false;
    }
    TestPointCloudRecords(*this, Cloud, *Frame);

    // Literal expectations (UE point (depth, (u - cx) d / fx, -(v - cy) d / fy), then y negated for ROS).
    struct FExpectedPoint
    {
        float X;
        float Y;
        float Z;
        int32 Ring;
    };
    const FExpectedPoint Expected[4] = {
        { 1.0f, 0.5f, 0.25f, 0 },   // pixel (0, 0), depth 1: UE y = -0.5
        { 2.0f, -1.0f, 0.5f, 0 },   // pixel (2, 0), depth 2: UE y = +1
        { 3.0f, 0.0f, -0.75f, 1 },  // pixel (1, 1), depth 3
        { 4.0f, -2.0f, -1.0f, 1 } }; // pixel (2, 1), depth 4: UE y = +2
    for (int32 Index = 0; Index < 4; ++Index)
    {
        const int32 Offset = Index * 22;
        TestEqual(FString::Printf(TEXT("Point %d x"), Index), ReadFloatLE(Cloud.data, Offset), Expected[Index].X, 0.0f);
        TestEqual(FString::Printf(TEXT("Point %d y is negated"), Index), ReadFloatLE(Cloud.data, Offset + 4), Expected[Index].Y, 0.0f);
        TestEqual(FString::Printf(TEXT("Point %d z"), Index), ReadFloatLE(Cloud.data, Offset + 8), Expected[Index].Z, 0.0f);
        TestEqual(FString::Printf(TEXT("Point %d intensity bytes are zero without intensity data"), Index), static_cast<int64>(ReadUInt32LE(Cloud.data, Offset + 12)), static_cast<int64>(0));
        TestEqual(FString::Printf(TEXT("Point %d ring"), Index), static_cast<int32>(ReadUInt16LE(Cloud.data, Offset + 16)), Expected[Index].Ring);
        TestEqual(FString::Printf(TEXT("Point %d time"), Index), ReadFloatLE(Cloud.data, Offset + 18), 0.0f, 0.0f);
    }

    // Byte order spot check: y of point 1 is -1.0f = 0xBF800000, stored little endian at offset 22 + 4.
    TestEqual(TEXT("float32 is little endian (byte 0)"), static_cast<int32>(Cloud.data[26]), 0x00);
    TestEqual(TEXT("float32 is little endian (byte 2)"), static_cast<int32>(Cloud.data[28]), 0x80);
    TestEqual(TEXT("float32 is little endian (byte 3)"), static_cast<int32>(Cloud.data[29]), 0xBF);
    // Ring 1 of point 2 as uint16 little endian at offset 44 + 16.
    TestEqual(TEXT("uint16 is little endian (low byte)"), static_cast<int32>(Cloud.data[60]), 1);
    TestEqual(TEXT("uint16 is little endian (high byte)"), static_cast<int32>(Cloud.data[61]), 0);

    // An empty cloud keeps the layout with no data.
    const FUBotRosPointCloud2 Empty = UBot::ROSConvert::MakePointCloud2(*NewObject<UUBotPointCloudFrame>(), TEXT("empty"));
    TestPointCloudLayout(*this, Empty, 0);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotROSConvertPointCloud2LidarTest, "UBotROS.Convert.PointCloud2Lidar", UBotROSConverterTests::TestFlags)

bool FUBotROSConvertPointCloud2LidarTest::RunTest(const FString& Parameters)
{
    using namespace UBotROSConverterTests;

    FScopedTestWorld TestWorld;
    if (!TestNotNull(TEXT("Test world"), TestWorld.Get()))
    {
        return false;
    }

    // Wall face at X = 4.5 m, 20 m wide, 6 m tall. With 8 columns over 360 degrees the -45, 0 and +45 degree
    // columns hit it on each of the three rings, so the cloud has returns with UE y < 0, = 0 and > 0.
    TestNotNull(TEXT("Wall"), TestWorld.SpawnBox(FVector(500.0, 0.0, 0.0), FVector(50.0, 1000.0, 300.0)));
    UUBot3DLidarSensorComponent* Lidar = TestWorld.SpawnLidar<UUBot3DLidarSensorComponent>();
    if (!TestNotNull(TEXT("3D LiDAR"), Lidar))
    {
        return false;
    }
    Lidar->NumberOfChannels = 3;
    Lidar->VerticalFovDownDegrees = 10.0f;
    Lidar->VerticalFovUpDegrees = 10.0f;
    Lidar->HorizontalFovDegrees = 360.0f;
    Lidar->HorizontalSamples = 8;
    Lidar->MaximumRangeMeters = 20.0f;

    TestTrue(TEXT("Scan succeeds"), Lidar->Sample3DLidar());
    const UUBotPointCloudFrame* Frame = Lidar->GetLatest3DLidarFrame();
    if (!TestNotNull(TEXT("Point cloud frame"), Frame))
    {
        return false;
    }
    TestTrue(TEXT("Precondition: LiDAR cloud carries intensities"), Frame->HasIntensityData());
    const int32 PointCount = Frame->GetPointCount();
    TestTrue(TEXT("Precondition: the wall produces returns"), PointCount > 0);

    const FUBotRosPointCloud2 Cloud = UBot::ROSConvert::MakePointCloud2(*Frame, TEXT("robot/lidar_3d"));
    TestEqual(TEXT("frame_id is the caller-resolved id"), *Cloud.header.frame_id, TEXT("robot/lidar_3d"));
    TestPointCloudLayout(*this, Cloud, PointCount);
    if (Cloud.data.Num() != PointCount * 22)
    {
        return false;
    }
    TestPointCloudRecords(*this, Cloud, *Frame);

    // The record checks are only meaningful with an off-axis return, a lit return and a ring above 0.
    bool bFoundRightReturn = false;
    bool bFoundUpperRing = false;
    for (int32 Index = 0; Index < PointCount; ++Index)
    {
        const int32 Offset = Index * 22;
        const FVector& UEPoint = Frame->GetPointsMeters()[Index];
        if (UEPoint.Y > 1.0)
        {
            bFoundRightReturn = true;
            TestTrue(TEXT("A return on the UE right has negative ROS y"), ReadFloatLE(Cloud.data, Offset + 4) < -1.0f);
            TestTrue(TEXT("A surface return has a positive intensity"), ReadFloatLE(Cloud.data, Offset + 12) > 0.0f);
        }
        bFoundUpperRing = bFoundUpperRing || ReadUInt16LE(Cloud.data, Offset + 16) == 2;
    }
    TestTrue(TEXT("Cloud contains a return on the UE right (+45 deg column)"), bFoundRightReturn);
    TestTrue(TEXT("Cloud contains ring 2"), bFoundUpperRing);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotROSConvertImageTest, "UBotROS.Convert.Image", UBotROSConverterTests::TestFlags)

bool FUBotROSConvertImageTest::RunTest(const FString& Parameters)
{
    using namespace UBotROSConverterTests;

    // BGRA8: two pixels in B, G, R, A byte order are copied unchanged.
    UUBotImageFrame* Color = NewObject<UUBotImageFrame>();
    Color->Width = 2;
    Color->Height = 1;
    Color->Encoding = EUBotImageEncoding::BGRA8;
    Color->Header.FrameId = TEXT("raw_camera");
    Color->Header.TimestampSeconds = 4.25;
    const TArray<uint8> ColorBytes = { 10, 20, 30, 255, 40, 50, 60, 128 };
    FUBotImageFrameTestAccess::PixelBytes(*Color) = ColorBytes;

    const FUBotRosImage ColorImage = UBot::ROSConvert::MakeImage(*Color, TEXT("robot/camera_optical"));
    TestEqual(TEXT("BGRA frame_id is the optical frame"), *ColorImage.header.frame_id, TEXT("robot/camera_optical"));
    TestStamp(*this, TEXT("BGRA stamp"), ColorImage.header.stamp, 4, 250000000);
    TestEqual(TEXT("BGRA encoding"), *ColorImage.encoding, TEXT("bgra8"));
    TestEqual(TEXT("BGRA width"), static_cast<int32>(ColorImage.width), 2);
    TestEqual(TEXT("BGRA height"), static_cast<int32>(ColorImage.height), 1);
    TestEqual(TEXT("BGRA step is width * 4"), static_cast<int32>(ColorImage.step), 8);
    TestEqual(TEXT("BGRA is little endian"), static_cast<int32>(ColorImage.is_bigendian), 0);
    TestTrue(TEXT("BGRA bytes are copied unchanged"), ColorImage.data == ColorBytes);

    // 32FC1: one little-endian float (meters) per pixel; invalid depth stays 0.
    UUBotImageFrame* Depth = NewObject<UUBotImageFrame>();
    Depth->Width = 3;
    Depth->Height = 2;
    Depth->Encoding = EUBotImageEncoding::Depth32FC1;
    Depth->Header.TimestampSeconds = 1.5;
    const TArray<float> DepthValues = { 1.0f, 0.0f, 2.5f, 0.125f, 3.0f, 40.0f };
    FUBotImageFrameTestAccess::DepthValuesMeters(*Depth) = DepthValues;

    const FUBotRosImage DepthImage = UBot::ROSConvert::MakeImage(*Depth, TEXT("robot/depth_optical"));
    TestEqual(TEXT("Depth frame_id is the optical frame"), *DepthImage.header.frame_id, TEXT("robot/depth_optical"));
    TestEqual(TEXT("Depth encoding"), *DepthImage.encoding, TEXT("32FC1"));
    TestEqual(TEXT("Depth width"), static_cast<int32>(DepthImage.width), 3);
    TestEqual(TEXT("Depth height"), static_cast<int32>(DepthImage.height), 2);
    TestEqual(TEXT("Depth step is width * 4"), static_cast<int32>(DepthImage.step), 12);
    TestEqual(TEXT("Depth is little endian"), static_cast<int32>(DepthImage.is_bigendian), 0);
    if (TestEqual(TEXT("Depth data is height * step bytes"), DepthImage.data.Num(), 24))
    {
        for (int32 Index = 0; Index < DepthValues.Num(); ++Index)
        {
            TestEqual(FString::Printf(TEXT("Depth pixel %d"), Index), ReadFloatLE(DepthImage.data, Index * 4), DepthValues[Index], 0.0f);
        }
        // 2.5f = 0x40200000, little endian at bytes 8..11.
        TestEqual(TEXT("Depth float is little endian (byte 2)"), static_cast<int32>(DepthImage.data[10]), 0x20);
        TestEqual(TEXT("Depth float is little endian (byte 3)"), static_cast<int32>(DepthImage.data[11]), 0x40);
    }

    // Without complete CPU pixel data the description is kept but no (partial) pixels are sent.
    UUBotImageFrame* NoPixels = NewObject<UUBotImageFrame>();
    NoPixels->Width = 4;
    NoPixels->Height = 2;
    NoPixels->Encoding = EUBotImageEncoding::BGRA8;
    FUBotImageFrameTestAccess::PixelBytes(*NoPixels) = { 1, 2, 3, 4 };
    const FUBotRosImage NoPixelImage = UBot::ROSConvert::MakeImage(*NoPixels, TEXT("camera_optical"));
    TestEqual(TEXT("Incomplete frame keeps its width"), static_cast<int32>(NoPixelImage.width), 4);
    TestEqual(TEXT("Incomplete frame keeps its step"), static_cast<int32>(NoPixelImage.step), 16);
    TestEqual(TEXT("Incomplete frame sends no pixel data"), NoPixelImage.data.Num(), 0);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotROSConvertCameraInfoTest, "UBotROS.Convert.CameraInfo", UBotROSConverterTests::TestFlags)

bool FUBotROSConvertCameraInfoTest::RunTest(const FString& Parameters)
{
    using namespace UBotROSConverterTests;

    UUBotImageFrame* Frame = NewObject<UUBotImageFrame>();
    Frame->Width = 640;
    Frame->Height = 480;
    Frame->Encoding = EUBotImageEncoding::BGRA8;
    Frame->Fx = 500.0f;
    Frame->Fy = 510.0f;
    Frame->Cx = 319.5f;
    Frame->Cy = 239.5f;
    Frame->Header.FrameId = TEXT("raw_camera");
    Frame->Header.TimestampSeconds = 7.5;

    const FUBotRosCameraInfo Info = UBot::ROSConvert::MakeCameraInfo(*Frame, TEXT("robot/camera_optical"));
    TestEqual(TEXT("frame_id is the optical frame"), *Info.header.frame_id, TEXT("robot/camera_optical"));
    TestStamp(*this, TEXT("CameraInfo stamp"), Info.header.stamp, 7, 500000000);
    TestEqual(TEXT("width"), static_cast<int32>(Info.width), 640);
    TestEqual(TEXT("height"), static_cast<int32>(Info.height), 480);
    TestEqual(TEXT("distortion_model"), *Info.distortion_model, TEXT("plumb_bob"));
    if (TestEqual(TEXT("plumb_bob has five coefficients"), Info.d.Num(), 5))
    {
        for (int32 Index = 0; Index < 5; ++Index)
        {
            TestEqual(FString::Printf(TEXT("d[%d] is zero"), Index), Info.d[Index], 0.0, 0.0);
        }
    }

    const double ExpectedK[9] = { 500.0, 0.0, 319.5, 0.0, 510.0, 239.5, 0.0, 0.0, 1.0 };
    const double ExpectedR[9] = { 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 };
    const double ExpectedP[12] = { 500.0, 0.0, 319.5, 0.0, 0.0, 510.0, 239.5, 0.0, 0.0, 0.0, 1.0, 0.0 };
    for (uint32 Index = 0; Index < 9; ++Index)
    {
        TestEqual(FString::Printf(TEXT("k[%u]"), Index), Info.k[Index], ExpectedK[Index], 0.0);
        TestEqual(FString::Printf(TEXT("r[%u]"), Index), Info.r[Index], ExpectedR[Index], 0.0);
    }
    for (uint32 Index = 0; Index < 12; ++Index)
    {
        TestEqual(FString::Printf(TEXT("p[%u]"), Index), Info.p[Index], ExpectedP[Index], 0.0);
    }
    TestEqual(TEXT("binning_x"), static_cast<int32>(Info.binning_x), 0);
    TestEqual(TEXT("binning_y"), static_cast<int32>(Info.binning_y), 0);
    TestEqual(TEXT("roi.x_offset"), static_cast<int32>(Info.roi.x_offset), 0);
    TestEqual(TEXT("roi.y_offset"), static_cast<int32>(Info.roi.y_offset), 0);
    TestEqual(TEXT("roi.width"), static_cast<int32>(Info.roi.width), 0);
    TestEqual(TEXT("roi.height"), static_cast<int32>(Info.roi.height), 0);
    TestFalse(TEXT("roi.do_rectify"), static_cast<bool>(Info.roi.do_rectify));
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotROSConvertTransformTest, "UBotROS.Convert.Transform", UBotROSConverterTests::TestFlags)

bool FUBotROSConvertTransformTest::RunTest(const FString& Parameters)
{
    using namespace UBotROSConverterTests;

    // Child 1 m forward, 2 m right, 3 m up of its parent, yawed 90 degrees to the right (UE).
    const FTransform YawedUE(FRotator(0.0, 90.0, 0.0), FVector(100.0, 200.0, 300.0));
    const FUBotRosTransformStamped Yawed = UBot::ROSConvert::MakeTransform(YawedUE, 12.5, TEXT("robot/base_link"), TEXT("robot/lidar"));
    TestEqual(TEXT("header.frame_id is the parent"), *Yawed.header.frame_id, TEXT("robot/base_link"));
    TestEqual(TEXT("child_frame_id is the child"), *Yawed.child_frame_id, TEXT("robot/lidar"));
    TestStamp(*this, TEXT("Transform stamp"), Yawed.header.stamp, 12, 500000000);
    TestEqual(TEXT("Translation in meters with y negated"), Yawed.transform.translation.ToVector(), FVector(1.0, -2.0, 3.0), 1.e-9f);
    const double HalfSqrt2 = FMath::Sqrt(0.5);
    TestTrue(TEXT("UE yaw right 90 deg is ROS yaw -90 deg"), QuatError(Yawed.transform.rotation.ToQuat(), FQuat(0.0, 0.0, -HalfSqrt2, HalfSqrt2)) < Tolerance);
    // The child's forward axis points to the parent's right, which is ROS -Y.
    const FVector ChildForwardInParent = ApplyRosTransform(Yawed.transform, FVector(1.0, 0.0, 0.0)) - Yawed.transform.translation.ToVector();
    TestEqual(TEXT("Child forward is parent right (ROS -Y)"), ChildForwardInParent, FVector(0.0, -1.0, 0.0), 1.e-9f);

    // General rotation: the ROS transform must map mirrored child points onto mirrored parent points.
    const FTransform GeneralUE(FQuat(FRotator(15.0, -40.0, 70.0)), FVector(-50.0, 120.0, 35.0));
    const FUBotRosTransformStamped General = UBot::ROSConvert::MakeTransform(GeneralUE, 0.0, TEXT("parent"), TEXT("child"));
    TestEqual(TEXT("Rotation is a unit quaternion"), General.transform.rotation.ToQuat().Size(), 1.0, Tolerance);
    const FVector ChildPointsUE[3] = { FVector(50.0, -20.0, 10.0), FVector(0.0, 0.0, 100.0), FVector(-30.0, 75.0, -5.0) };
    for (int32 Index = 0; Index < 3; ++Index)
    {
        const FVector ParentPointUE = GeneralUE.TransformPosition(ChildPointsUE[Index]);
        const FVector ExpectedRos(ParentPointUE.X / 100.0, -ParentPointUE.Y / 100.0, ParentPointUE.Z / 100.0);
        const FVector ChildPointRos(ChildPointsUE[Index].X / 100.0, -ChildPointsUE[Index].Y / 100.0, ChildPointsUE[Index].Z / 100.0);
        TestEqual(FString::Printf(TEXT("Child point %d maps to the mirrored parent point"), Index), ApplyRosTransform(General.transform, ChildPointRos), ExpectedRos, 1.e-9f);
    }
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotROSConvertOpticalTransformTest, "UBotROS.Convert.OpticalTransform", UBotROSConverterTests::TestFlags)

bool FUBotROSConvertOpticalTransformTest::RunTest(const FString& Parameters)
{
    using namespace UBotROSConverterTests;

    const FUBotRosTransformStamped Optical = UBot::ROSConvert::MakeOpticalTransform(3.25, TEXT("robot/camera"), TEXT("robot/camera_optical"));
    TestEqual(TEXT("header.frame_id is the camera frame"), *Optical.header.frame_id, TEXT("robot/camera"));
    TestEqual(TEXT("child_frame_id is the optical frame"), *Optical.child_frame_id, TEXT("robot/camera_optical"));
    TestStamp(*this, TEXT("Optical stamp"), Optical.header.stamp, 3, 250000000);
    TestEqual(TEXT("No translation"), Optical.transform.translation.ToVector(), FVector::ZeroVector, 0.0f);
    TestEqual(TEXT("rotation.x"), Optical.transform.rotation.x, -0.5, 0.0);
    TestEqual(TEXT("rotation.y"), Optical.transform.rotation.y, 0.5, 0.0);
    TestEqual(TEXT("rotation.z"), Optical.transform.rotation.z, -0.5, 0.0);
    TestEqual(TEXT("rotation.w"), Optical.transform.rotation.w, 0.5, 0.0);

    // REP-103 optical axes in camera link coordinates: z forward (+X), x right (-Y), y down (-Z).
    TestEqual(TEXT("Optical z looks forward"), ApplyRosTransform(Optical.transform, FVector(0.0, 0.0, 1.0)), FVector(1.0, 0.0, 0.0), 1.e-12f);
    TestEqual(TEXT("Optical x points right"), ApplyRosTransform(Optical.transform, FVector(1.0, 0.0, 0.0)), FVector(0.0, -1.0, 0.0), 1.e-12f);
    TestEqual(TEXT("Optical y points down"), ApplyRosTransform(Optical.transform, FVector(0.0, 1.0, 0.0)), FVector(0.0, 0.0, -1.0), 1.e-12f);

    // Chained with a camera that looks to the UE right, the optical axis points along base ROS -Y.
    const FUBotRosTransformStamped Camera = UBot::ROSConvert::MakeTransform(FTransform(FRotator(0.0, 90.0, 0.0), FVector::ZeroVector), 3.25, TEXT("base_link"), TEXT("robot/camera"));
    const FVector OpticalAxisInBase = ApplyRosTransform(Camera.transform, ApplyRosTransform(Optical.transform, FVector(0.0, 0.0, 1.0)));
    TestEqual(TEXT("Right-looking camera sees along base -Y"), OpticalAxisInBase, FVector(0.0, -1.0, 0.0), 1.e-9f);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotROSConvertVelocityCommandTest, "UBotROS.Convert.VelocityCommand", UBotROSConverterTests::TestFlags)

bool FUBotROSConvertVelocityCommandTest::RunTest(const FString& Parameters)
{
    using namespace UBotROSConverterTests;

    // ROS: +y is left, +angular.z turns left. UE: +Y is right, +Z turns right.
    FUBotRosTwist Twist;
    Twist.linear.x = 1.5;
    Twist.linear.y = 1.0;
    Twist.linear.z = 0.25;
    Twist.angular.x = 0.1;
    Twist.angular.y = 0.2;
    Twist.angular.z = 1.0;

    const FUBotVelocityCommand Command = UBot::ROSConvert::MakeVelocityCommand(Twist, 3.5);
    TestEqual(TEXT("linear.y = +1 (left) is UE Y = -1"), Command.LinearVelocityMetersPerSecond.Y, -1.0, Tolerance);
    TestEqual(TEXT("angular.z = +1 (turn left) is UE Z = -1"), Command.AngularVelocityRadiansPerSecond.Z, -1.0, Tolerance);
    TestEqual(TEXT("Linear is (x, -y, z) in m/s"), Command.LinearVelocityMetersPerSecond, FVector(1.5, -1.0, 0.25), 1.e-9f);
    TestEqual(TEXT("Angular is (-x, y, -z) in rad/s"), Command.AngularVelocityRadiansPerSecond, FVector(-0.1, 0.2, -1.0), 1.e-9f);
    TestEqual(TEXT("Timestamp is passed through"), Command.TimestampSeconds, 3.5, 0.0);
    TestTrue(TEXT("Source is ROS"), Command.Source == FName(TEXT("ROS")));

    // Mapping back with the UE -> ROS conventions restores the twist.
    TestEqual(TEXT("Linear round trip"), UBot::Ros::VectorUEToROS(Command.LinearVelocityMetersPerSecond), FVector(1.5, 1.0, 0.25), 1.e-9f);
    TestEqual(TEXT("Angular round trip"), UBot::Ros::AngularVelocityUEToROS(Command.AngularVelocityRadiansPerSecond), FVector(0.1, 0.2, 1.0), 1.e-9f);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotROSConvertAttitudeTargetTest, "UBotROS.Convert.AttitudeTarget", UBotROSConverterTests::TestFlags)

bool FUBotROSConvertAttitudeTargetTest::RunTest(const FString& Parameters)
{
    using namespace UBotROSConverterTests;
    bool bBodyRate = true;
    FUBotAttitudeCommand Attitude;
    FUBotBodyRateCommand Rates;

    // Attitude: yaw +90 degrees in ROS (nose to the left, +Y) is the nose along UE -Y.
    FUBotRosAttitudeTarget Target;
    const FQuat RosYawLeft(FVector::UpVector, UE_DOUBLE_HALF_PI);
    Target.orientation = FUBotRosQuaternion::FromQuat(RosYawLeft);
    Target.body_rate.z = 3.0;
    Target.thrust = 0.75f;
    TestTrue(TEXT("Attitude converts"), UBot::ROSConvert::MakeAttitudeTargetCommand(Target, 2.0, bBodyRate, Attitude, Rates));
    TestFalse(TEXT("Attitude, not body rate"), bBodyRate);
    TestTrue(FString::Printf(TEXT("Nose along UE -Y (%s)"), *Attitude.Orientation.GetAxisX().ToString()), Attitude.Orientation.GetAxisX().Equals(FVector(0.0, -1.0, 0.0), 1e-9));
    TestTrue(TEXT("Thrust used"), Attitude.bUseThrust);
    TestEqual(TEXT("Thrust"), Attitude.NormalizedThrust, 0.75, 1e-9);
    TestFalse(TEXT("Body rate fields do not turn the heading"), Attitude.bUseYawRate);
    TestEqual(TEXT("Timestamp"), Attitude.TimestampSeconds, 2.0, 0.0);
    TestTrue(TEXT("Source is ROS"), Attitude.Source == FName(TEXT("ROS")));

    // Any rotation goes through the uBotCore conventions.
    Target.orientation = FUBotRosQuaternion::FromQuat(FQuat(FVector::ForwardVector, 0.3));
    UBot::ROSConvert::MakeAttitudeTargetCommand(Target, 0.0, bBodyRate, Attitude, Rates);
    TestTrue(TEXT("Rotation through RotationROSToUE"), Attitude.Orientation.Equals(UBot::Ros::RotationROSToUE(FQuat(FVector::ForwardVector, 0.3)), 1e-12));

    // Without thrust the vehicle holds its altitude; thrust is clamped.
    Target.type_mask = FUBotRosAttitudeTarget::IGNORE_THRUST;
    UBot::ROSConvert::MakeAttitudeTargetCommand(Target, 0.0, bBodyRate, Attitude, Rates);
    TestFalse(TEXT("IGNORE_THRUST: no thrust"), Attitude.bUseThrust);
    TestEqual(TEXT("IGNORE_THRUST: hold altitude"), Attitude.VerticalVelocityMetersPerSecond, 0.0);
    Target.type_mask = 0;
    Target.thrust = 1.5f;
    UBot::ROSConvert::MakeAttitudeTargetCommand(Target, 0.0, bBodyRate, Attitude, Rates);
    TestEqual(TEXT("Thrust clamped to 1"), Attitude.NormalizedThrust, 1.0, 0.0);

    // Body rates: FLU -> UE (-x, y, -z); ignored axes are 0.
    Target.type_mask = FUBotRosAttitudeTarget::IGNORE_ATTITUDE | FUBotRosAttitudeTarget::IGNORE_PITCH_RATE;
    Target.body_rate = FUBotRosVector3::FromVector(FVector(0.5, 0.25, 1.0));
    Target.thrust = 0.5f;
    TestTrue(TEXT("Body rate converts"), UBot::ROSConvert::MakeAttitudeTargetCommand(Target, 0.0, bBodyRate, Attitude, Rates));
    TestTrue(TEXT("Body rate"), bBodyRate);
    TestTrue(FString::Printf(TEXT("Rates (%s)"), *Rates.BodyRateRadiansPerSecond.ToString()), Rates.BodyRateRadiansPerSecond.Equals(FVector(-0.5, 0.0, -1.0), 1e-12));
    TestEqual(TEXT("Body-rate thrust"), Rates.NormalizedThrust, 0.5, 1e-9);
    Target.type_mask = FUBotRosAttitudeTarget::IGNORE_ATTITUDE | FUBotRosAttitudeTarget::IGNORE_ROLL_RATE | FUBotRosAttitudeTarget::IGNORE_YAW_RATE;
    UBot::ROSConvert::MakeAttitudeTargetCommand(Target, 0.0, bBodyRate, Attitude, Rates);
    TestTrue(TEXT("Only pitch rate"), Rates.BodyRateRadiansPerSecond.Equals(FVector(0.0, 0.25, 0.0), 1e-12));

    // Invalid values are rejected.
    FString Error;
    FUBotRosAttitudeTarget Zero;
    Zero.orientation = FUBotRosQuaternion::FromQuat(FQuat(0.0, 0.0, 0.0, 0.0));
    TestFalse(TEXT("Zero quaternion rejected"), UBot::ROSConvert::MakeAttitudeTargetCommand(Zero, 0.0, bBodyRate, Attitude, Rates, &Error));
    TestFalse(TEXT("With a reason"), Error.IsEmpty());
    FUBotRosAttitudeTarget NotFinite;
    NotFinite.thrust = std::numeric_limits<float>::quiet_NaN();
    TestFalse(TEXT("NaN thrust rejected"), UBot::ROSConvert::MakeAttitudeTargetCommand(NotFinite, 0.0, bBodyRate, Attitude, Rates));
    NotFinite.thrust = 0.5f;
    NotFinite.type_mask = FUBotRosAttitudeTarget::IGNORE_ATTITUDE;
    NotFinite.body_rate.y = std::numeric_limits<double>::infinity();
    TestFalse(TEXT("Infinite rate rejected"), UBot::ROSConvert::MakeAttitudeTargetCommand(NotFinite, 0.0, bBodyRate, Attitude, Rates));
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotROSConvertActuatorsTest, "UBotROS.Convert.Actuators", UBotROSConverterTests::TestFlags)

bool FUBotROSConvertActuatorsTest::RunTest(const FString& Parameters)
{
    FUBotMotorCommand Command;
    FUBotRosActuators Actuators;
    Actuators.velocity = { 800.0, -5.0, 810.5, 0.0 };
    Actuators.normalized = { 0.5, 0.5, 0.5, 0.5 };
    TestTrue(TEXT("Velocities convert"), UBot::ROSConvert::MakeMotorCommand(Actuators, 1.25, Command));
    TestEqual(TEXT("Velocity wins over normalized"), Command.Type, EUBotMotorCommandType::AngularVelocity);
    TestTrue(TEXT("Negative speed counts as 0"), Command.Values == TArray<double>({ 800.0, 0.0, 810.5, 0.0 }));
    TestEqual(TEXT("Timestamp"), Command.TimestampSeconds, 1.25, 0.0);

    Actuators.velocity.Reset();
    Actuators.normalized = { 0.25, 1.5, -0.5 };
    TestTrue(TEXT("Normalized converts"), UBot::ROSConvert::MakeMotorCommand(Actuators, 0.0, Command));
    TestEqual(TEXT("Normalized type"), Command.Type, EUBotMotorCommandType::Normalized);
    TestTrue(TEXT("Clamped to 0..1"), Command.Values == TArray<double>({ 0.25, 1.0, 0.0 }));

    FString Error;
    Actuators.normalized.Reset();
    Actuators.position = { 1.0 };
    TestFalse(TEXT("Position only is rejected"), UBot::ROSConvert::MakeMotorCommand(Actuators, 0.0, Command, &Error));
    TestFalse(TEXT("With a reason"), Error.IsEmpty());
    Actuators.normalized = { 0.5, std::numeric_limits<double>::quiet_NaN() };
    TestFalse(TEXT("NaN rejected"), UBot::ROSConvert::MakeMotorCommand(Actuators, 0.0, Command));
    return true;
}

#endif
