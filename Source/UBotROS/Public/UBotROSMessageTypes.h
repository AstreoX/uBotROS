#pragma once

#include "CoreMinimal.h"
#include "Containers/StaticArray.h"
#include "UBotCdr.h"

/**
 * Plain C++ mirrors of the ROS 2 Humble messages used by uBot, field for field and in declaration
 * order of the .msg files, so that Serialize() produces exactly the CDR bytes of the ROS 2 middleware.
 * Field names are the ROS field names on purpose (snake_case), which keeps the mapping to the .msg
 * files obvious. Defaults are the ROS defaults (zero, empty, identity quaternion).
 *
 * The structs hold ROS data: values are expected in ROS conventions (meters, radians, REP-103 axes).
 * Converting from UE is the caller's job (UBot::RightHanded / UBot::Units from uBotCore, and
 * UBot::ROSConventions for time and names); the FromVector / FromQuat helpers
 * below copy components verbatim and perform no conversion.
 *
 * Every struct provides:
 *   static const TCHAR* TypeName();               // e.g. TEXT("sensor_msgs/msg/Imu")
 *   void Serialize(FUBotCdrWriter& Writer) const;  // fields only, no encapsulation header
 *   bool Deserialize(FUBotCdrReader& Reader);      // false on truncated or malformed data
 * Use UBot::Ros::Encode / UBot::Ros::Decode for complete messages.
 */

/** builtin_interfaces/msg/Time */
struct UBOTROS_API FUBotRosTime
{
    int32 sec = 0;
    uint32 nanosec = 0;

    static const TCHAR* TypeName()
    {
        return TEXT("builtin_interfaces/msg/Time");
    }

    /** Seconds -> (sec, nanosec) through UBot::ROSConventions::SecondsToRosTime (negative or non-finite -> 0). */
    static FUBotRosTime FromSeconds(double Seconds);

    /** (sec, nanosec) -> seconds through UBot::ROSConventions::RosTimeToSeconds. */
    double ToSeconds() const;

    void Serialize(FUBotCdrWriter& Writer) const;
    bool Deserialize(FUBotCdrReader& Reader);
};

/** std_msgs/msg/Header */
struct UBOTROS_API FUBotRosHeader
{
    FUBotRosTime stamp;
    FString frame_id;

    static const TCHAR* TypeName()
    {
        return TEXT("std_msgs/msg/Header");
    }

    void Serialize(FUBotCdrWriter& Writer) const;
    bool Deserialize(FUBotCdrReader& Reader);
};

/** geometry_msgs/msg/Point */
struct UBOTROS_API FUBotRosPoint
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    static const TCHAR* TypeName()
    {
        return TEXT("geometry_msgs/msg/Point");
    }

    /** Copies the components verbatim (no unit or axis conversion). */
    static FUBotRosPoint FromVector(const FVector& Value)
    {
        FUBotRosPoint Result;
        Result.x = Value.X;
        Result.y = Value.Y;
        Result.z = Value.Z;
        return Result;
    }

    FVector ToVector() const
    {
        return FVector(x, y, z);
    }

    void Serialize(FUBotCdrWriter& Writer) const;
    bool Deserialize(FUBotCdrReader& Reader);
};

/** geometry_msgs/msg/Vector3 */
struct UBOTROS_API FUBotRosVector3
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    static const TCHAR* TypeName()
    {
        return TEXT("geometry_msgs/msg/Vector3");
    }

    /** Copies the components verbatim (no unit or axis conversion). */
    static FUBotRosVector3 FromVector(const FVector& Value)
    {
        FUBotRosVector3 Result;
        Result.x = Value.X;
        Result.y = Value.Y;
        Result.z = Value.Z;
        return Result;
    }

    FVector ToVector() const
    {
        return FVector(x, y, z);
    }

    void Serialize(FUBotCdrWriter& Writer) const;
    bool Deserialize(FUBotCdrReader& Reader);
};

/** geometry_msgs/msg/Quaternion (default is the identity, as in ROS). */
struct UBOTROS_API FUBotRosQuaternion
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double w = 1.0;

    static const TCHAR* TypeName()
    {
        return TEXT("geometry_msgs/msg/Quaternion");
    }

    /** Copies the components verbatim (no axis conversion). */
    static FUBotRosQuaternion FromQuat(const FQuat& Value)
    {
        FUBotRosQuaternion Result;
        Result.x = Value.X;
        Result.y = Value.Y;
        Result.z = Value.Z;
        Result.w = Value.W;
        return Result;
    }

    FQuat ToQuat() const
    {
        return FQuat(x, y, z, w);
    }

    void Serialize(FUBotCdrWriter& Writer) const;
    bool Deserialize(FUBotCdrReader& Reader);
};

/** geometry_msgs/msg/Pose */
struct UBOTROS_API FUBotRosPose
{
    FUBotRosPoint position;
    FUBotRosQuaternion orientation;

    static const TCHAR* TypeName()
    {
        return TEXT("geometry_msgs/msg/Pose");
    }

    void Serialize(FUBotCdrWriter& Writer) const;
    bool Deserialize(FUBotCdrReader& Reader);
};

/** geometry_msgs/msg/Transform */
struct UBOTROS_API FUBotRosTransform
{
    FUBotRosVector3 translation;
    FUBotRosQuaternion rotation;

    static const TCHAR* TypeName()
    {
        return TEXT("geometry_msgs/msg/Transform");
    }

    void Serialize(FUBotCdrWriter& Writer) const;
    bool Deserialize(FUBotCdrReader& Reader);
};

/** geometry_msgs/msg/TransformStamped */
struct UBOTROS_API FUBotRosTransformStamped
{
    FUBotRosHeader header;
    FString child_frame_id;
    FUBotRosTransform transform;

    static const TCHAR* TypeName()
    {
        return TEXT("geometry_msgs/msg/TransformStamped");
    }

    void Serialize(FUBotCdrWriter& Writer) const;
    bool Deserialize(FUBotCdrReader& Reader);
};

/** geometry_msgs/msg/PoseStamped */
struct UBOTROS_API FUBotRosPoseStamped
{
    FUBotRosHeader header;
    FUBotRosPose pose;

    static const TCHAR* TypeName()
    {
        return TEXT("geometry_msgs/msg/PoseStamped");
    }

    void Serialize(FUBotCdrWriter& Writer) const;
    bool Deserialize(FUBotCdrReader& Reader);
};

/** geometry_msgs/msg/PoseWithCovariance (covariance: row-major 6x6 over x, y, z, roll, pitch, yaw). */
struct UBOTROS_API FUBotRosPoseWithCovariance
{
    FUBotRosPose pose;
    TStaticArray<double, 36> covariance{InPlace, 0.0};

    static const TCHAR* TypeName()
    {
        return TEXT("geometry_msgs/msg/PoseWithCovariance");
    }

    void Serialize(FUBotCdrWriter& Writer) const;
    bool Deserialize(FUBotCdrReader& Reader);
};

/** geometry_msgs/msg/Twist */
struct UBOTROS_API FUBotRosTwist
{
    FUBotRosVector3 linear;
    FUBotRosVector3 angular;

    static const TCHAR* TypeName()
    {
        return TEXT("geometry_msgs/msg/Twist");
    }

    void Serialize(FUBotCdrWriter& Writer) const;
    bool Deserialize(FUBotCdrReader& Reader);
};

/** geometry_msgs/msg/TwistWithCovariance (covariance: row-major 6x6). */
struct UBOTROS_API FUBotRosTwistWithCovariance
{
    FUBotRosTwist twist;
    TStaticArray<double, 36> covariance{InPlace, 0.0};

    static const TCHAR* TypeName()
    {
        return TEXT("geometry_msgs/msg/TwistWithCovariance");
    }

    void Serialize(FUBotCdrWriter& Writer) const;
    bool Deserialize(FUBotCdrReader& Reader);
};

/** nav_msgs/msg/Odometry */
struct UBOTROS_API FUBotRosOdometry
{
    FUBotRosHeader header;
    FString child_frame_id;
    FUBotRosPoseWithCovariance pose;
    FUBotRosTwistWithCovariance twist;

    static const TCHAR* TypeName()
    {
        return TEXT("nav_msgs/msg/Odometry");
    }

    void Serialize(FUBotCdrWriter& Writer) const;
    bool Deserialize(FUBotCdrReader& Reader);
};

/** sensor_msgs/msg/Imu (covariances row-major 3x3; element 0 = -1 marks "not provided" per ROS). */
struct UBOTROS_API FUBotRosImu
{
    FUBotRosHeader header;
    FUBotRosQuaternion orientation;
    TStaticArray<double, 9> orientation_covariance{InPlace, 0.0};
    FUBotRosVector3 angular_velocity;
    TStaticArray<double, 9> angular_velocity_covariance{InPlace, 0.0};
    FUBotRosVector3 linear_acceleration;
    TStaticArray<double, 9> linear_acceleration_covariance{InPlace, 0.0};

    static const TCHAR* TypeName()
    {
        return TEXT("sensor_msgs/msg/Imu");
    }

    void Serialize(FUBotCdrWriter& Writer) const;
    bool Deserialize(FUBotCdrReader& Reader);
};

/** sensor_msgs/msg/LaserScan */
struct UBOTROS_API FUBotRosLaserScan
{
    FUBotRosHeader header;
    float angle_min = 0.0f;
    float angle_max = 0.0f;
    float angle_increment = 0.0f;
    float time_increment = 0.0f;
    float scan_time = 0.0f;
    float range_min = 0.0f;
    float range_max = 0.0f;
    TArray<float> ranges;
    TArray<float> intensities;

    static const TCHAR* TypeName()
    {
        return TEXT("sensor_msgs/msg/LaserScan");
    }

    void Serialize(FUBotCdrWriter& Writer) const;
    bool Deserialize(FUBotCdrReader& Reader);
};

/** sensor_msgs/msg/PointField */
struct UBOTROS_API FUBotRosPointField
{
    // datatype values
    static constexpr uint8 INT8 = 1;
    static constexpr uint8 UINT8 = 2;
    static constexpr uint8 INT16 = 3;
    static constexpr uint8 UINT16 = 4;
    static constexpr uint8 INT32 = 5;
    static constexpr uint8 UINT32 = 6;
    static constexpr uint8 FLOAT32 = 7;
    static constexpr uint8 FLOAT64 = 8;

    FString name;
    uint32 offset = 0;
    uint8 datatype = 0;
    uint32 count = 0;

    static const TCHAR* TypeName()
    {
        return TEXT("sensor_msgs/msg/PointField");
    }

    /** Convenience constructor for one field entry. */
    static FUBotRosPointField Make(const FString& InName, uint32 InOffset, uint8 InDatatype, uint32 InCount = 1)
    {
        FUBotRosPointField Result;
        Result.name = InName;
        Result.offset = InOffset;
        Result.datatype = InDatatype;
        Result.count = InCount;
        return Result;
    }

    void Serialize(FUBotCdrWriter& Writer) const;
    bool Deserialize(FUBotCdrReader& Reader);
};

/** sensor_msgs/msg/PointCloud2 */
struct UBOTROS_API FUBotRosPointCloud2
{
    FUBotRosHeader header;
    uint32 height = 0;
    uint32 width = 0;
    TArray<FUBotRosPointField> fields;
    bool is_bigendian = false;
    uint32 point_step = 0;
    uint32 row_step = 0;
    TArray<uint8> data;
    bool is_dense = false;

    static const TCHAR* TypeName()
    {
        return TEXT("sensor_msgs/msg/PointCloud2");
    }

    void Serialize(FUBotCdrWriter& Writer) const;
    bool Deserialize(FUBotCdrReader& Reader);
};

/** sensor_msgs/msg/Image (is_bigendian is uint8 in the .msg file). */
struct UBOTROS_API FUBotRosImage
{
    FUBotRosHeader header;
    uint32 height = 0;
    uint32 width = 0;
    FString encoding;
    uint8 is_bigendian = 0;
    uint32 step = 0;
    TArray<uint8> data;

    static const TCHAR* TypeName()
    {
        return TEXT("sensor_msgs/msg/Image");
    }

    void Serialize(FUBotCdrWriter& Writer) const;
    bool Deserialize(FUBotCdrReader& Reader);
};

/** sensor_msgs/msg/RegionOfInterest */
struct UBOTROS_API FUBotRosRegionOfInterest
{
    uint32 x_offset = 0;
    uint32 y_offset = 0;
    uint32 height = 0;
    uint32 width = 0;
    bool do_rectify = false;

    static const TCHAR* TypeName()
    {
        return TEXT("sensor_msgs/msg/RegionOfInterest");
    }

    void Serialize(FUBotCdrWriter& Writer) const;
    bool Deserialize(FUBotCdrReader& Reader);
};

/** sensor_msgs/msg/CameraInfo (k 3x3, r 3x3, p 3x4, all row-major). */
struct UBOTROS_API FUBotRosCameraInfo
{
    FUBotRosHeader header;
    uint32 height = 0;
    uint32 width = 0;
    FString distortion_model;
    TArray<double> d;
    TStaticArray<double, 9> k{InPlace, 0.0};
    TStaticArray<double, 9> r{InPlace, 0.0};
    TStaticArray<double, 12> p{InPlace, 0.0};
    uint32 binning_x = 0;
    uint32 binning_y = 0;
    FUBotRosRegionOfInterest roi;

    static const TCHAR* TypeName()
    {
        return TEXT("sensor_msgs/msg/CameraInfo");
    }

    void Serialize(FUBotCdrWriter& Writer) const;
    bool Deserialize(FUBotCdrReader& Reader);
};

/** tf2_msgs/msg/TFMessage */
struct UBOTROS_API FUBotRosTfMessage
{
    TArray<FUBotRosTransformStamped> transforms;

    static const TCHAR* TypeName()
    {
        return TEXT("tf2_msgs/msg/TFMessage");
    }

    void Serialize(FUBotCdrWriter& Writer) const;
    bool Deserialize(FUBotCdrReader& Reader);
};

/** rosgraph_msgs/msg/Clock */
struct UBOTROS_API FUBotRosClock
{
    FUBotRosTime clock;

    static const TCHAR* TypeName()
    {
        return TEXT("rosgraph_msgs/msg/Clock");
    }

    void Serialize(FUBotCdrWriter& Writer) const;
    bool Deserialize(FUBotCdrReader& Reader);
};

/** mavros_msgs/msg/AttitudeTarget (orientation in the ROS world frame, body_rate in FLU). */
struct UBOTROS_API FUBotRosAttitudeTarget
{
    static constexpr uint8 IGNORE_ROLL_RATE = 1;
    static constexpr uint8 IGNORE_PITCH_RATE = 2;
    static constexpr uint8 IGNORE_YAW_RATE = 4;
    static constexpr uint8 IGNORE_THRUST = 64;
    static constexpr uint8 IGNORE_ATTITUDE = 128;

    FUBotRosHeader header;
    uint8 type_mask = 0;
    FUBotRosQuaternion orientation;
    FUBotRosVector3 body_rate;
    float thrust = 0.0f;

    static const TCHAR* TypeName()
    {
        return TEXT("mavros_msgs/msg/AttitudeTarget");
    }

    void Serialize(FUBotCdrWriter& Writer) const;
    bool Deserialize(FUBotCdrReader& Reader);
};

/** actuator_msgs/msg/Actuators */
struct UBOTROS_API FUBotRosActuators
{
    FUBotRosHeader header;
    TArray<double> position;
    TArray<double> velocity;
    TArray<double> normalized;

    static const TCHAR* TypeName()
    {
        return TEXT("actuator_msgs/msg/Actuators");
    }

    void Serialize(FUBotCdrWriter& Writer) const;
    bool Deserialize(FUBotCdrReader& Reader);
};

namespace UBot::Ros
{
    /** Serializes a complete message: encapsulation header + CDR body, ready for a PUBLISH frame. */
    template <typename T>
    TArray<uint8> Encode(const T& Msg)
    {
        FUBotCdrWriter Writer;
        Msg.Serialize(Writer);
        return Writer.Finish();
    }

    /**
     * Decodes a complete message (encapsulation header + CDR body, trailing bytes ignored). Returns
     * false on an invalid header, truncated or malformed data; OutMsg must not be used in that case.
     */
    template <typename T>
    bool Decode(TConstArrayView<uint8> Cdr, T& OutMsg)
    {
        FUBotCdrReader Reader(Cdr);
        return Reader.IsOk() && OutMsg.Deserialize(Reader) && Reader.IsOk();
    }
}
