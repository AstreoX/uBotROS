#include "UBotROSConverters.h"

#include "UBot2DLidarFrame.h"
#include "UBotImageFrame.h"
#include "UBotPointCloudFrame.h"
#include "UBotRosConventions.h"

#include <limits>

namespace UBot::ROSConvert::Private
{
    constexpr int32 CovarianceDimension = 6;
    constexpr int32 CovarianceSize = CovarianceDimension * CovarianceDimension;
    constexpr uint32 ImageBytesPerPixel = 4;

    template <uint32 NumElements>
    void AssignArray(TStaticArray<double, NumElements>& Destination, const double (&Values)[NumElements])
    {
        for (uint32 Index = 0; Index < NumElements; ++Index)
        {
            Destination[Index] = Values[Index];
        }
    }

    /** Row-major 3x3 matrix with Variance on the diagonal (indices 0, 4, 8) and zeros elsewhere. */
    void FillDiagonal3(const FVector& Variance, TStaticArray<double, 9>& OutMatrix)
    {
        const double Values[9] = {
            Variance.X, 0.0, 0.0,
            0.0, Variance.Y, 0.0,
            0.0, 0.0, Variance.Z };
        AssignArray(OutMatrix, Values);
    }

    uint32 ToUnsigned(int32 Value)
    {
        return Value > 0 ? static_cast<uint32>(Value) : 0u;
    }

    void WriteUInt16LE(uint8* Destination, uint16 Value)
    {
        Destination[0] = static_cast<uint8>(Value & 0xFFu);
        Destination[1] = static_cast<uint8>((Value >> 8) & 0xFFu);
    }

    void WriteUInt32LE(uint8* Destination, uint32 Value)
    {
        Destination[0] = static_cast<uint8>(Value & 0xFFu);
        Destination[1] = static_cast<uint8>((Value >> 8) & 0xFFu);
        Destination[2] = static_cast<uint8>((Value >> 16) & 0xFFu);
        Destination[3] = static_cast<uint8>((Value >> 24) & 0xFFu);
    }

    void WriteFloatLE(uint8* Destination, float Value)
    {
        uint32 Bits = 0;
        static_assert(sizeof(Bits) == sizeof(Value), "float must be 32 bits");
        FMemory::Memcpy(&Bits, &Value, sizeof(Value));
        WriteUInt32LE(Destination, Bits);
    }
}

namespace UBot::ROSConvert
{
    FUBotRosHeader MakeHeader(const FUBotFrameHeader& Header, const FString& FrameId)
    {
        FUBotRosHeader Result;
        Result.stamp = FUBotRosTime::FromSeconds(Header.TimestampSeconds);
        Result.frame_id = FrameId;
        return Result;
    }

    FUBotRosPoseStamped MakePose(const FUBotPoseSensorData& Data, const FString& FrameId)
    {
        FUBotRosPoseStamped Result;
        Result.header = MakeHeader(Data.Header, FrameId);
        Result.pose.position = FUBotRosPoint::FromVector(UBot::Ros::VectorUEToROS(Data.PositionMeters));
        Result.pose.orientation = FUBotRosQuaternion::FromQuat(UBot::Ros::RotationUEToROS(Data.OrientationQuaternion));
        return Result;
    }

    FUBotRosOdometry MakeOdometry(const FUBotOdometrySensorData& Data, const FString& OdomFrameId, const FString& ChildFrameId)
    {
        FUBotRosOdometry Result;
        Result.header = MakeHeader(Data.Header, OdomFrameId);
        Result.child_frame_id = ChildFrameId;
        Result.pose.pose.position = FUBotRosPoint::FromVector(UBot::Ros::VectorUEToROS(Data.PositionMeters));
        Result.pose.pose.orientation = FUBotRosQuaternion::FromQuat(UBot::Ros::RotationUEToROS(Data.OrientationQuaternion));
        ConvertCovariance6(Data.PoseCovariance, Result.pose.covariance);
        Result.twist.twist.linear = FUBotRosVector3::FromVector(UBot::Ros::VectorUEToROS(Data.LinearVelocityLocalMetersPerSecond));
        Result.twist.twist.angular = FUBotRosVector3::FromVector(UBot::Ros::AngularVelocityUEToROS(Data.AngularVelocityLocalRadPerSecond));
        ConvertCovariance6(Data.TwistCovariance, Result.twist.covariance);
        return Result;
    }

    FUBotRosImu MakeImu(const FUBotImuSensorData& Data, const FString& FrameId)
    {
        FUBotRosImu Result;
        Result.header = MakeHeader(Data.Header, FrameId);
        Result.orientation = FUBotRosQuaternion::FromQuat(UBot::Ros::RotationUEToROS(Data.OrientationQuaternion));
        Private::FillDiagonal3(Data.OrientationVariance, Result.orientation_covariance);
        Result.angular_velocity = FUBotRosVector3::FromVector(UBot::Ros::AngularVelocityUEToROS(Data.AngularVelocityRadPerSecond));
        Private::FillDiagonal3(Data.AngularVelocityVariance, Result.angular_velocity_covariance);
        Result.linear_acceleration = FUBotRosVector3::FromVector(UBot::Ros::VectorUEToROS(Data.LinearAccelerationMetersPerSecondSquared));
        Private::FillDiagonal3(Data.LinearAccelerationVariance, Result.linear_acceleration_covariance);
        return Result;
    }

    FUBotRosLaserScan MakeLaserScan(const UUBot2DLidarFrame& Frame, const FString& FrameId)
    {
        const TArray<float>& Ranges = Frame.GetRangesMeters();
        const TArray<uint8>& HitFlags = Frame.GetHitFlags();
        const TArray<float>& Intensities = Frame.GetIntensities();
        const int32 SampleCount = Ranges.Num();
        const bool bHasIntensities = SampleCount > 0 && Intensities.Num() == SampleCount;

        FUBotRosLaserScan Scan;
        Scan.header = MakeHeader(Frame.Header, FrameId);
        // UE sample k looks along AngleMin + k * Increment, positive to the right. Mirrored into ROS
        // (positive to the left) the last UE sample has the smallest ROS angle, so the scan starts there.
        Scan.angle_min = static_cast<float>(UBot::Ros::YawUEToROS(Frame.AngleMaxRadians));
        Scan.angle_max = static_cast<float>(UBot::Ros::YawUEToROS(Frame.AngleMinRadians));
        Scan.angle_increment = Frame.AngleIncrementRadians;
        Scan.time_increment = SampleCount > 1 ? Frame.ScanPeriodSeconds / static_cast<float>(SampleCount - 1) : 0.0f;
        Scan.scan_time = Frame.ScanPeriodSeconds;
        Scan.range_min = Frame.RangeMinMeters;
        Scan.range_max = Frame.RangeMaxMeters;

        Scan.ranges.SetNumUninitialized(SampleCount);
        if (bHasIntensities)
        {
            Scan.intensities.SetNumUninitialized(SampleCount);
        }
        for (int32 RosIndex = 0; RosIndex < SampleCount; ++RosIndex)
        {
            const int32 UEIndex = SampleCount - 1 - RosIndex;
            const bool bHit = HitFlags.IsValidIndex(UEIndex) && HitFlags[UEIndex] != 0;
            Scan.ranges[RosIndex] = bHit ? Ranges[UEIndex] : std::numeric_limits<float>::infinity();
            if (bHasIntensities)
            {
                Scan.intensities[RosIndex] = Intensities[UEIndex];
            }
        }
        return Scan;
    }

    FUBotRosPointCloud2 MakePointCloud2(const UUBotPointCloudFrame& Frame, const FString& FrameId)
    {
        const TArray<FVector>& Points = Frame.GetPointsMeters();
        const TArray<int32>& Rings = Frame.GetRingIndices();
        const TArray<float>& TimeOffsets = Frame.GetTimeOffsetsSeconds();
        const TArray<float>& Intensities = Frame.GetIntensities();
        const bool bHasIntensity = Frame.HasIntensityData();

        FUBotRosPointCloud2 Cloud;
        Cloud.header = MakeHeader(Frame.Header, FrameId);
        Cloud.fields.Reserve(6);
        Cloud.fields.Add(FUBotRosPointField::Make(TEXT("x"), PointCloudOffsetX, FUBotRosPointField::FLOAT32));
        Cloud.fields.Add(FUBotRosPointField::Make(TEXT("y"), PointCloudOffsetY, FUBotRosPointField::FLOAT32));
        Cloud.fields.Add(FUBotRosPointField::Make(TEXT("z"), PointCloudOffsetZ, FUBotRosPointField::FLOAT32));
        Cloud.fields.Add(FUBotRosPointField::Make(TEXT("intensity"), PointCloudOffsetIntensity, FUBotRosPointField::FLOAT32));
        Cloud.fields.Add(FUBotRosPointField::Make(TEXT("ring"), PointCloudOffsetRing, FUBotRosPointField::UINT16));
        Cloud.fields.Add(FUBotRosPointField::Make(TEXT("time"), PointCloudOffsetTime, FUBotRosPointField::FLOAT32));
        Cloud.is_bigendian = false;
        Cloud.point_step = PointCloudPointStep;
        Cloud.height = 1;

        // A payload beyond int32 cannot be held by TArray<uint8>; such a cloud is sent empty instead.
        const int64 DataSize = static_cast<int64>(Points.Num()) * PointCloudPointStep;
        const int32 PointCount = DataSize <= MAX_int32 ? Points.Num() : 0;
        Cloud.width = static_cast<uint32>(PointCount);
        Cloud.row_step = PointCloudPointStep * Cloud.width;
        Cloud.data.SetNumUninitialized(PointCount * static_cast<int32>(PointCloudPointStep));

        bool bDense = true;
        uint8* Record = Cloud.data.GetData();
        for (int32 Index = 0; Index < PointCount; ++Index, Record += PointCloudPointStep)
        {
            const FVector RosPoint = UBot::Ros::VectorUEToROS(Points[Index]);
            const float X = static_cast<float>(RosPoint.X);
            const float Y = static_cast<float>(RosPoint.Y);
            const float Z = static_cast<float>(RosPoint.Z);
            const float Intensity = bHasIntensity ? Intensities[Index] : 0.0f;
            const int32 Ring = Rings.IsValidIndex(Index) ? Rings[Index] : 0;
            const float Time = TimeOffsets.IsValidIndex(Index) ? TimeOffsets[Index] : 0.0f;

            Private::WriteFloatLE(Record + PointCloudOffsetX, X);
            Private::WriteFloatLE(Record + PointCloudOffsetY, Y);
            Private::WriteFloatLE(Record + PointCloudOffsetZ, Z);
            Private::WriteFloatLE(Record + PointCloudOffsetIntensity, Intensity);
            Private::WriteUInt16LE(Record + PointCloudOffsetRing, static_cast<uint16>(FMath::Clamp<int32>(Ring, 0, MAX_uint16)));
            Private::WriteFloatLE(Record + PointCloudOffsetTime, Time);
            bDense = bDense && FMath::IsFinite(X) && FMath::IsFinite(Y) && FMath::IsFinite(Z);
        }
        Cloud.is_dense = bDense;
        return Cloud;
    }

    FUBotRosImage MakeImage(const UUBotImageFrame& Frame, const FString& OpticalFrameId)
    {
        const uint32 Width = Private::ToUnsigned(Frame.Width);
        const bool bDepth = Frame.Encoding == EUBotImageEncoding::Depth32FC1;

        FUBotRosImage Image;
        Image.header = MakeHeader(Frame.Header, OpticalFrameId);
        Image.height = Private::ToUnsigned(Frame.Height);
        Image.width = Width;
        Image.encoding = bDepth ? ImageEncodingDepth32FC1 : ImageEncodingBGRA8;
        Image.is_bigendian = 0;
        // Both encodings carry 4 bytes per pixel and the frame stores its rows tightly packed.
        Image.step = Width * Private::ImageBytesPerPixel;

        if (!Frame.HasCPUPixelData())
        {
            return Image;
        }
        if (!bDepth)
        {
            Image.data = Frame.GetPixelBytes();
            return Image;
        }

        const TArray<float>& Depths = Frame.GetDepthValuesMeters();
        Image.data.SetNumUninitialized(Depths.Num() * static_cast<int32>(sizeof(float)));
#if PLATFORM_LITTLE_ENDIAN
        FMemory::Memcpy(Image.data.GetData(), Depths.GetData(), Image.data.Num());
#else
        for (int32 Index = 0; Index < Depths.Num(); ++Index)
        {
            Private::WriteFloatLE(Image.data.GetData() + Index * sizeof(float), Depths[Index]);
        }
#endif
        return Image;
    }

    FUBotRosCameraInfo MakeCameraInfo(const UUBotImageFrame& Frame, const FString& OpticalFrameId)
    {
        const double Fx = Frame.Fx;
        const double Fy = Frame.Fy;
        const double Cx = Frame.Cx;
        const double Cy = Frame.Cy;

        FUBotRosCameraInfo Info;
        Info.header = MakeHeader(Frame.Header, OpticalFrameId);
        Info.height = Private::ToUnsigned(Frame.Height);
        Info.width = Private::ToUnsigned(Frame.Width);
        Info.distortion_model = TEXT("plumb_bob");
        Info.d.Init(0.0, 5);

        const double K[9] = {
            Fx, 0.0, Cx,
            0.0, Fy, Cy,
            0.0, 0.0, 1.0 };
        const double R[9] = {
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0 };
        const double P[12] = {
            Fx, 0.0, Cx, 0.0,
            0.0, Fy, Cy, 0.0,
            0.0, 0.0, 1.0, 0.0 };
        Private::AssignArray(Info.k, K);
        Private::AssignArray(Info.r, R);
        Private::AssignArray(Info.p, P);

        Info.binning_x = 0;
        Info.binning_y = 0;
        Info.roi = FUBotRosRegionOfInterest();
        return Info;
    }

    FUBotRosTransformStamped MakeTransform(const FTransform& ParentFromChildUE, double StampSeconds, const FString& ParentFrame, const FString& ChildFrame)
    {
        const FTransform ParentFromChildROS = UBot::Ros::TransformUEToROS(ParentFromChildUE);

        FUBotRosTransformStamped Result;
        Result.header.stamp = FUBotRosTime::FromSeconds(StampSeconds);
        Result.header.frame_id = ParentFrame;
        Result.child_frame_id = ChildFrame;
        Result.transform.translation = FUBotRosVector3::FromVector(ParentFromChildROS.GetTranslation());
        Result.transform.rotation = FUBotRosQuaternion::FromQuat(ParentFromChildROS.GetRotation());
        return Result;
    }

    FUBotRosTransformStamped MakeOpticalTransform(double StampSeconds, const FString& CameraFrame, const FString& OpticalFrame)
    {
        FUBotRosTransformStamped Result;
        Result.header.stamp = FUBotRosTime::FromSeconds(StampSeconds);
        Result.header.frame_id = CameraFrame;
        Result.child_frame_id = OpticalFrame;
        Result.transform.translation = FUBotRosVector3::FromVector(FVector::ZeroVector);
        Result.transform.rotation = FUBotRosQuaternion::FromQuat(UBot::Ros::CameraLinkToOpticalRotation());
        return Result;
    }

    void ConvertCovariance6(const TArray<double>& UECovariance, TStaticArray<double, 36>& OutROS)
    {
        const bool bValid = UECovariance.Num() == Private::CovarianceSize;
        for (int32 Row = 0; Row < Private::CovarianceDimension; ++Row)
        {
            for (int32 Column = 0; Column < Private::CovarianceDimension; ++Column)
            {
                const int32 Index = Row * Private::CovarianceDimension + Column;
                // S = diag(+, -, +, -, +, -): s_r * s_c is negative exactly when one of r, c is odd.
                const double Sign = ((Row + Column) % 2 == 0) ? 1.0 : -1.0;
                OutROS[static_cast<uint32>(Index)] = bValid ? Sign * UECovariance[Index] : 0.0;
            }
        }
    }

    FUBotVelocityCommand MakeVelocityCommand(const FUBotRosTwist& Twist, double TimestampSeconds)
    {
        static const FName RosSource(TEXT("ROS"));

        FUBotVelocityCommand Command;
        Command.LinearVelocityMetersPerSecond = UBot::Ros::VectorROSToUE(Twist.linear.ToVector());
        Command.AngularVelocityRadiansPerSecond = UBot::Ros::AngularVelocityROSToUE(Twist.angular.ToVector());
        Command.TimestampSeconds = TimestampSeconds;
        Command.Source = RosSource;
        return Command;
    }

    bool MakeAttitudeTargetCommand(const FUBotRosAttitudeTarget& Target, double TimestampSeconds, bool& bOutBodyRate,
        FUBotAttitudeCommand& OutAttitude, FUBotBodyRateCommand& OutBodyRate, FString* OutError)
    {
        static const FName RosSource(TEXT("ROS"));
        const auto Fail = [OutError](const TCHAR* Why)
        {
            if (OutError)
            {
                *OutError = Why;
            }
            return false;
        };

        const bool bUseThrust = (Target.type_mask & FUBotRosAttitudeTarget::IGNORE_THRUST) == 0;
        if (bUseThrust && !FMath::IsFinite(Target.thrust))
        {
            return Fail(TEXT("thrust is not finite"));
        }
        const double Thrust = bUseThrust ? FMath::Clamp(static_cast<double>(Target.thrust), 0.0, 1.0) : 0.0;

        bOutBodyRate = (Target.type_mask & FUBotRosAttitudeTarget::IGNORE_ATTITUDE) != 0;
        if (bOutBodyRate)
        {
            FVector Rates = Target.body_rate.ToVector();
            if (Rates.ContainsNaN())
            {
                return Fail(TEXT("body_rate is not finite"));
            }
            if (Target.type_mask & FUBotRosAttitudeTarget::IGNORE_ROLL_RATE)
            {
                Rates.X = 0.0;
            }
            if (Target.type_mask & FUBotRosAttitudeTarget::IGNORE_PITCH_RATE)
            {
                Rates.Y = 0.0;
            }
            if (Target.type_mask & FUBotRosAttitudeTarget::IGNORE_YAW_RATE)
            {
                Rates.Z = 0.0;
            }
            OutBodyRate = FUBotBodyRateCommand();
            OutBodyRate.BodyRateRadiansPerSecond = UBot::Ros::AngularVelocityROSToUE(Rates);
            OutBodyRate.bUseThrust = bUseThrust;
            OutBodyRate.NormalizedThrust = Thrust;
            OutBodyRate.TimestampSeconds = TimestampSeconds;
            OutBodyRate.Source = RosSource;
            return true;
        }

        const FQuat Orientation = Target.orientation.ToQuat();
        if (!FMath::IsFinite(Orientation.X) || !FMath::IsFinite(Orientation.Y) || !FMath::IsFinite(Orientation.Z) || !FMath::IsFinite(Orientation.W))
        {
            return Fail(TEXT("orientation is not finite"));
        }
        if (Orientation.SizeSquared() < 0.01)
        {
            return Fail(TEXT("orientation is not a rotation (zero quaternion)"));
        }
        OutAttitude = FUBotAttitudeCommand();
        OutAttitude.Orientation = UBot::Ros::RotationROSToUE(Orientation.GetNormalized());
        OutAttitude.bUseThrust = bUseThrust;
        OutAttitude.NormalizedThrust = Thrust;
        OutAttitude.TimestampSeconds = TimestampSeconds;
        OutAttitude.Source = RosSource;
        return true;
    }

    bool MakeMotorCommand(const FUBotRosActuators& Actuators, double TimestampSeconds, FUBotMotorCommand& OutCommand, FString* OutError)
    {
        static const FName RosSource(TEXT("ROS"));
        const bool bSpeeds = !Actuators.velocity.IsEmpty();
        const TArray<double>& Values = bSpeeds ? Actuators.velocity : Actuators.normalized;
        if (Values.IsEmpty())
        {
            if (OutError)
            {
                *OutError = TEXT("neither velocity nor normalized has values");
            }
            return false;
        }
        OutCommand = FUBotMotorCommand();
        OutCommand.Type = bSpeeds ? EUBotMotorCommandType::AngularVelocity : EUBotMotorCommandType::Normalized;
        OutCommand.Values.Reserve(Values.Num());
        for (const double Value : Values)
        {
            if (!FMath::IsFinite(Value))
            {
                if (OutError)
                {
                    *OutError = bSpeeds ? TEXT("velocity has a value that is not finite") : TEXT("normalized has a value that is not finite");
                }
                return false;
            }
            OutCommand.Values.Add(bSpeeds ? FMath::Max(Value, 0.0) : FMath::Clamp(Value, 0.0, 1.0));
        }
        OutCommand.TimestampSeconds = TimestampSeconds;
        OutCommand.Source = RosSource;
        return true;
    }
}
