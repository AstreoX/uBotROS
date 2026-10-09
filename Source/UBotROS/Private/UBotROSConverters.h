#pragma once

#include "CoreMinimal.h"
#include "Containers/StaticArray.h"
#include "UBotActuatorInterface.h"
#include "UBotROSMessageTypes.h"
#include "UBotSensorTypes.h"
#include "UBotTypes.h"

class UUBot2DLidarFrame;
class UUBotImageFrame;
class UUBotPointCloudFrame;

/**
 * uBot data -> ROS 2 message structs (ros-contract section 6), and ROS cmd_vel -> uBot velocity command.
 *
 * Pure functions without I/O or global state. Frame ids are resolved by the caller (namespace prefix,
 * optical suffix, overrides); the converters never read frame ids out of the uBot data. Every coordinate,
 * unit and time conversion goes through UBot::Ros (UBotCore), see UBotRosConventions.h for the math.
 */
namespace UBot::ROSConvert
{
    /** sensor_msgs/PointCloud2 point layout written by MakePointCloud2 (little endian, packed, 22 bytes). */
    inline constexpr uint32 PointCloudOffsetX = 0;
    inline constexpr uint32 PointCloudOffsetY = 4;
    inline constexpr uint32 PointCloudOffsetZ = 8;
    inline constexpr uint32 PointCloudOffsetIntensity = 12;
    inline constexpr uint32 PointCloudOffsetRing = 16;
    inline constexpr uint32 PointCloudOffsetTime = 18;
    inline constexpr uint32 PointCloudPointStep = 22;

    /** sensor_msgs/Image encodings used by MakeImage. */
    inline constexpr const TCHAR* ImageEncodingBGRA8 = TEXT("bgra8");
    inline constexpr const TCHAR* ImageEncodingDepth32FC1 = TEXT("32FC1");

    /**
     * std_msgs/Header: stamp = FUBotRosTime::FromSeconds(Header.TimestampSeconds) (UBot::Ros::SecondsToRosTime),
     * frame_id = FrameId (Header.FrameId is ignored).
     */
    FUBotRosHeader MakeHeader(const FUBotFrameHeader& Header, const FString& FrameId);

    /** position = VectorUEToROS(PositionMeters), orientation = RotationUEToROS. Velocity is not part of PoseStamped. */
    FUBotRosPoseStamped MakePose(const FUBotPoseSensorData& Data, const FString& FrameId);

    /**
     * header.frame_id = OdomFrameId, child_frame_id = ChildFrameId. Pose as MakePose; twist.linear via
     * VectorUEToROS, twist.angular via AngularVelocityUEToROS; both 6x6 covariances via ConvertCovariance6.
     */
    FUBotRosOdometry MakeOdometry(const FUBotOdometrySensorData& Data, const FString& OdomFrameId, const FString& ChildFrameId);

    /**
     * orientation = RotationUEToROS, angular_velocity = AngularVelocityUEToROS (UE yaw-right rate -> negative
     * ROS z), linear_acceleration = VectorUEToROS. The per-axis variances fill the covariance diagonals
     * (indices 0, 4, 8); a mirror only flips signs, so variances are unchanged. Off-diagonal entries are 0.
     */
    FUBotRosImu MakeImu(const FUBotImuSensorData& Data, const FString& FrameId);

    /**
     * Mirrored scan: UE angles grow to the right, ROS angles to the left. angle_min = YawUEToROS(AngleMax),
     * angle_max = YawUEToROS(AngleMin), angle_increment unchanged (positive), and ROS index i holds UE sample
     * N - 1 - i, so ROS angle_min + i * angle_increment = -(UE angle of that sample). Samples without a hit
     * report +inf (REP 117). Intensities are reversed the same way; they are empty when the frame carries
     * none. scan_time = ScanPeriodSeconds, time_increment = ScanPeriodSeconds / (N - 1) (legacy behaviour).
     */
    FUBotRosLaserScan MakeLaserScan(const UUBot2DLidarFrame& Frame, const FString& FrameId);

    /**
     * Unorganized cloud (height 1, width = point count) with fields x, y, z (float32 @ 0/4/8), intensity
     * (float32 @ 12), ring (uint16 @ 16), time (float32 @ 18), point_step 22, little endian. Points go through
     * VectorUEToROS (sensor-local meters, y negated); intensity is 0 for frames without intensity data; ring
     * is clamped to [0, 65535]. is_dense is true when every coordinate is finite.
     */
    FUBotRosPointCloud2 MakePointCloud2(const UUBotPointCloudFrame& Frame, const FString& FrameId);

    /**
     * BGRA8 frames -> "bgra8", Depth32FC1 frames -> "32FC1" (meters, little endian), step = Width * 4,
     * header.frame_id = OpticalFrameId. The pixels are copied only when the frame holds complete CPU data
     * (UUBotImageFrame::HasCPUPixelData); otherwise data stays empty and the caller must not publish the
     * message (call RequestPixelReadback first). Invalid depth pixels keep the frame's value (0).
     */
    FUBotRosImage MakeImage(const UUBotImageFrame& Frame, const FString& OpticalFrameId);

    /**
     * Ideal pinhole camera on the optical frame: distortion_model "plumb_bob" with d = 5 zeros,
     * K = [Fx 0 Cx; 0 Fy Cy; 0 0 1], R = identity, P = [Fx 0 Cx 0; 0 Fy Cy 0; 0 0 1 0], no binning, empty ROI.
     */
    FUBotRosCameraInfo MakeCameraInfo(const UUBotImageFrame& Frame, const FString& OpticalFrameId);

    /**
     * ParentFromChild is the child's transform expressed in the parent (UE, centimeters), i.e. it maps child
     * coordinates to parent coordinates. Converted with UBot::Ros::TransformUEToROS (meters, mirrored
     * rotation, scale dropped). header.frame_id = ParentFrame, child_frame_id = ChildFrame.
     */
    FUBotRosTransformStamped MakeTransform(const FTransform& ParentFromChildUE, double StampSeconds, const FString& ParentFrame, const FString& ChildFrame);

    /** CameraFrame -> OpticalFrame: zero translation, rotation UBot::Ros::CameraLinkToOpticalRotation(). */
    FUBotRosTransformStamped MakeOpticalTransform(double StampSeconds, const FString& CameraFrame, const FString& OpticalFrame);

    /**
     * 6x6 row-major covariance over (x, y, z, rot_x, rot_y, rot_z) or (vx, vy, vz, wx, wy, wz): the mirror
     * S = diag(+, -, +, -, +, -) (polar part (x, -y, z), axial part (-x, y, -z)) gives C' = S C S, so entry
     * (r, c) flips sign when exactly one of r, c is odd. An input that does not have 36 entries yields zeros.
     */
    void ConvertCovariance6(const TArray<double>& UECovariance, TStaticArray<double, 36>& OutROS);

    /**
     * geometry_msgs/Twist (ROS body frame) -> FUBotVelocityCommand (UE body axes): linear via VectorROSToUE,
     * angular via AngularVelocityROSToUE (ROS angular.z = +1 -> UE Z = -1), Source "ROS". No clamping.
     */
    FUBotVelocityCommand MakeVelocityCommand(const FUBotRosTwist& Twist, double TimestampSeconds);
}
