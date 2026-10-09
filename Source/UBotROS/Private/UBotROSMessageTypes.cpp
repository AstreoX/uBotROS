#include "UBotROSMessageTypes.h"

#include "UBotRosConventions.h"

namespace UBotROSMessageTypesPrivate
{
    // Smallest possible CDR size of one sequence element. They bound the allocations a corrupt
    // sequence count can cause (see FUBotCdrReader::ReadSequenceLength).
    // TransformStamped: stamp 8 + frame_id length 4 + child_frame_id length 4 + transform 7 * 8.
    constexpr int32 MinTransformStampedBytes = 8 + 4 + 4 + 7 * 8;
    // PointField: name length 4 + offset 4 + datatype 1 + count 4.
    constexpr int32 MinPointFieldBytes = 4 + 4 + 1 + 4;

    /** Reads a sequence of messages of type T. */
    template <typename T>
    bool ReadMessageSequence(FUBotCdrReader& Reader, TArray<T>& OutValues, int32 MinElementBytes)
    {
        OutValues.Reset();
        int32 Count = 0;
        if (!Reader.ReadSequenceLength(Count, MinElementBytes))
        {
            return false;
        }

        OutValues.Reserve(Count);
        for (int32 Index = 0; Index < Count; ++Index)
        {
            if (!OutValues.AddDefaulted_GetRef().Deserialize(Reader))
            {
                OutValues.Reset();
                return false;
            }
        }
        return true;
    }

    /** Writes a sequence of messages of type T. */
    template <typename T>
    void WriteMessageSequence(FUBotCdrWriter& Writer, const TArray<T>& Values)
    {
        Writer.WriteSequenceLength(Values.Num());
        for (const T& Value : Values)
        {
            Value.Serialize(Writer);
        }
    }
}

// ---------------------------------------------------------------------------------------------
// builtin_interfaces

FUBotRosTime FUBotRosTime::FromSeconds(double Seconds)
{
    FUBotRosTime Result;
    UBot::Ros::SecondsToRosTime(Seconds, Result.sec, Result.nanosec);
    return Result;
}

double FUBotRosTime::ToSeconds() const
{
    return UBot::Ros::RosTimeToSeconds(sec, nanosec);
}

void FUBotRosTime::Serialize(FUBotCdrWriter& Writer) const
{
    Writer.WriteI32(sec);
    Writer.WriteU32(nanosec);
}

bool FUBotRosTime::Deserialize(FUBotCdrReader& Reader)
{
    Reader.ReadI32(sec);
    Reader.ReadU32(nanosec);
    return Reader.IsOk();
}

// ---------------------------------------------------------------------------------------------
// std_msgs

void FUBotRosHeader::Serialize(FUBotCdrWriter& Writer) const
{
    stamp.Serialize(Writer);
    Writer.WriteString(frame_id);
}

bool FUBotRosHeader::Deserialize(FUBotCdrReader& Reader)
{
    stamp.Deserialize(Reader);
    Reader.ReadString(frame_id);
    return Reader.IsOk();
}

// ---------------------------------------------------------------------------------------------
// geometry_msgs

void FUBotRosPoint::Serialize(FUBotCdrWriter& Writer) const
{
    Writer.WriteF64(x);
    Writer.WriteF64(y);
    Writer.WriteF64(z);
}

bool FUBotRosPoint::Deserialize(FUBotCdrReader& Reader)
{
    Reader.ReadF64(x);
    Reader.ReadF64(y);
    Reader.ReadF64(z);
    return Reader.IsOk();
}

void FUBotRosVector3::Serialize(FUBotCdrWriter& Writer) const
{
    Writer.WriteF64(x);
    Writer.WriteF64(y);
    Writer.WriteF64(z);
}

bool FUBotRosVector3::Deserialize(FUBotCdrReader& Reader)
{
    Reader.ReadF64(x);
    Reader.ReadF64(y);
    Reader.ReadF64(z);
    return Reader.IsOk();
}

void FUBotRosQuaternion::Serialize(FUBotCdrWriter& Writer) const
{
    Writer.WriteF64(x);
    Writer.WriteF64(y);
    Writer.WriteF64(z);
    Writer.WriteF64(w);
}

bool FUBotRosQuaternion::Deserialize(FUBotCdrReader& Reader)
{
    Reader.ReadF64(x);
    Reader.ReadF64(y);
    Reader.ReadF64(z);
    Reader.ReadF64(w);
    return Reader.IsOk();
}

void FUBotRosPose::Serialize(FUBotCdrWriter& Writer) const
{
    position.Serialize(Writer);
    orientation.Serialize(Writer);
}

bool FUBotRosPose::Deserialize(FUBotCdrReader& Reader)
{
    position.Deserialize(Reader);
    orientation.Deserialize(Reader);
    return Reader.IsOk();
}

void FUBotRosTransform::Serialize(FUBotCdrWriter& Writer) const
{
    translation.Serialize(Writer);
    rotation.Serialize(Writer);
}

bool FUBotRosTransform::Deserialize(FUBotCdrReader& Reader)
{
    translation.Deserialize(Reader);
    rotation.Deserialize(Reader);
    return Reader.IsOk();
}

void FUBotRosTransformStamped::Serialize(FUBotCdrWriter& Writer) const
{
    header.Serialize(Writer);
    Writer.WriteString(child_frame_id);
    transform.Serialize(Writer);
}

bool FUBotRosTransformStamped::Deserialize(FUBotCdrReader& Reader)
{
    header.Deserialize(Reader);
    Reader.ReadString(child_frame_id);
    transform.Deserialize(Reader);
    return Reader.IsOk();
}

void FUBotRosPoseStamped::Serialize(FUBotCdrWriter& Writer) const
{
    header.Serialize(Writer);
    pose.Serialize(Writer);
}

bool FUBotRosPoseStamped::Deserialize(FUBotCdrReader& Reader)
{
    header.Deserialize(Reader);
    pose.Deserialize(Reader);
    return Reader.IsOk();
}

void FUBotRosPoseWithCovariance::Serialize(FUBotCdrWriter& Writer) const
{
    pose.Serialize(Writer);
    Writer.WriteF64Array(covariance);
}

bool FUBotRosPoseWithCovariance::Deserialize(FUBotCdrReader& Reader)
{
    pose.Deserialize(Reader);
    Reader.ReadF64Array(covariance);
    return Reader.IsOk();
}

void FUBotRosTwist::Serialize(FUBotCdrWriter& Writer) const
{
    linear.Serialize(Writer);
    angular.Serialize(Writer);
}

bool FUBotRosTwist::Deserialize(FUBotCdrReader& Reader)
{
    linear.Deserialize(Reader);
    angular.Deserialize(Reader);
    return Reader.IsOk();
}

void FUBotRosTwistWithCovariance::Serialize(FUBotCdrWriter& Writer) const
{
    twist.Serialize(Writer);
    Writer.WriteF64Array(covariance);
}

bool FUBotRosTwistWithCovariance::Deserialize(FUBotCdrReader& Reader)
{
    twist.Deserialize(Reader);
    Reader.ReadF64Array(covariance);
    return Reader.IsOk();
}

// ---------------------------------------------------------------------------------------------
// nav_msgs

void FUBotRosOdometry::Serialize(FUBotCdrWriter& Writer) const
{
    header.Serialize(Writer);
    Writer.WriteString(child_frame_id);
    pose.Serialize(Writer);
    twist.Serialize(Writer);
}

bool FUBotRosOdometry::Deserialize(FUBotCdrReader& Reader)
{
    header.Deserialize(Reader);
    Reader.ReadString(child_frame_id);
    pose.Deserialize(Reader);
    twist.Deserialize(Reader);
    return Reader.IsOk();
}

// ---------------------------------------------------------------------------------------------
// sensor_msgs

void FUBotRosImu::Serialize(FUBotCdrWriter& Writer) const
{
    header.Serialize(Writer);
    orientation.Serialize(Writer);
    Writer.WriteF64Array(orientation_covariance);
    angular_velocity.Serialize(Writer);
    Writer.WriteF64Array(angular_velocity_covariance);
    linear_acceleration.Serialize(Writer);
    Writer.WriteF64Array(linear_acceleration_covariance);
}

bool FUBotRosImu::Deserialize(FUBotCdrReader& Reader)
{
    header.Deserialize(Reader);
    orientation.Deserialize(Reader);
    Reader.ReadF64Array(orientation_covariance);
    angular_velocity.Deserialize(Reader);
    Reader.ReadF64Array(angular_velocity_covariance);
    linear_acceleration.Deserialize(Reader);
    Reader.ReadF64Array(linear_acceleration_covariance);
    return Reader.IsOk();
}

void FUBotRosLaserScan::Serialize(FUBotCdrWriter& Writer) const
{
    Writer.Reserve(64 + (static_cast<int64>(ranges.Num()) + intensities.Num()) * static_cast<int64>(sizeof(float)));
    header.Serialize(Writer);
    Writer.WriteF32(angle_min);
    Writer.WriteF32(angle_max);
    Writer.WriteF32(angle_increment);
    Writer.WriteF32(time_increment);
    Writer.WriteF32(scan_time);
    Writer.WriteF32(range_min);
    Writer.WriteF32(range_max);
    Writer.WriteF32Sequence(ranges);
    Writer.WriteF32Sequence(intensities);
}

bool FUBotRosLaserScan::Deserialize(FUBotCdrReader& Reader)
{
    header.Deserialize(Reader);
    Reader.ReadF32(angle_min);
    Reader.ReadF32(angle_max);
    Reader.ReadF32(angle_increment);
    Reader.ReadF32(time_increment);
    Reader.ReadF32(scan_time);
    Reader.ReadF32(range_min);
    Reader.ReadF32(range_max);
    Reader.ReadF32Sequence(ranges);
    Reader.ReadF32Sequence(intensities);
    return Reader.IsOk();
}

void FUBotRosPointField::Serialize(FUBotCdrWriter& Writer) const
{
    Writer.WriteString(name);
    Writer.WriteU32(offset);
    Writer.WriteU8(datatype);
    Writer.WriteU32(count);
}

bool FUBotRosPointField::Deserialize(FUBotCdrReader& Reader)
{
    Reader.ReadString(name);
    Reader.ReadU32(offset);
    Reader.ReadU8(datatype);
    Reader.ReadU32(count);
    return Reader.IsOk();
}

void FUBotRosPointCloud2::Serialize(FUBotCdrWriter& Writer) const
{
    Writer.Reserve(128 + static_cast<int64>(fields.Num()) * 32 + data.Num());
    header.Serialize(Writer);
    Writer.WriteU32(height);
    Writer.WriteU32(width);
    UBotROSMessageTypesPrivate::WriteMessageSequence(Writer, fields);
    Writer.WriteBool(is_bigendian);
    Writer.WriteU32(point_step);
    Writer.WriteU32(row_step);
    Writer.WriteU8Sequence(data);
    Writer.WriteBool(is_dense);
}

bool FUBotRosPointCloud2::Deserialize(FUBotCdrReader& Reader)
{
    header.Deserialize(Reader);
    Reader.ReadU32(height);
    Reader.ReadU32(width);
    UBotROSMessageTypesPrivate::ReadMessageSequence(Reader, fields, UBotROSMessageTypesPrivate::MinPointFieldBytes);
    Reader.ReadBool(is_bigendian);
    Reader.ReadU32(point_step);
    Reader.ReadU32(row_step);
    Reader.ReadU8Sequence(data);
    Reader.ReadBool(is_dense);
    return Reader.IsOk();
}

void FUBotRosImage::Serialize(FUBotCdrWriter& Writer) const
{
    Writer.Reserve(64 + static_cast<int64>(header.frame_id.Len() + encoding.Len()) * 4 + data.Num());
    header.Serialize(Writer);
    Writer.WriteU32(height);
    Writer.WriteU32(width);
    Writer.WriteString(encoding);
    Writer.WriteU8(is_bigendian);
    Writer.WriteU32(step);
    Writer.WriteU8Sequence(data);
}

bool FUBotRosImage::Deserialize(FUBotCdrReader& Reader)
{
    header.Deserialize(Reader);
    Reader.ReadU32(height);
    Reader.ReadU32(width);
    Reader.ReadString(encoding);
    Reader.ReadU8(is_bigendian);
    Reader.ReadU32(step);
    Reader.ReadU8Sequence(data);
    return Reader.IsOk();
}

void FUBotRosRegionOfInterest::Serialize(FUBotCdrWriter& Writer) const
{
    Writer.WriteU32(x_offset);
    Writer.WriteU32(y_offset);
    Writer.WriteU32(height);
    Writer.WriteU32(width);
    Writer.WriteBool(do_rectify);
}

bool FUBotRosRegionOfInterest::Deserialize(FUBotCdrReader& Reader)
{
    Reader.ReadU32(x_offset);
    Reader.ReadU32(y_offset);
    Reader.ReadU32(height);
    Reader.ReadU32(width);
    Reader.ReadBool(do_rectify);
    return Reader.IsOk();
}

void FUBotRosCameraInfo::Serialize(FUBotCdrWriter& Writer) const
{
    header.Serialize(Writer);
    Writer.WriteU32(height);
    Writer.WriteU32(width);
    Writer.WriteString(distortion_model);
    Writer.WriteF64Sequence(d);
    Writer.WriteF64Array(k);
    Writer.WriteF64Array(r);
    Writer.WriteF64Array(p);
    Writer.WriteU32(binning_x);
    Writer.WriteU32(binning_y);
    roi.Serialize(Writer);
}

bool FUBotRosCameraInfo::Deserialize(FUBotCdrReader& Reader)
{
    header.Deserialize(Reader);
    Reader.ReadU32(height);
    Reader.ReadU32(width);
    Reader.ReadString(distortion_model);
    Reader.ReadF64Sequence(d);
    Reader.ReadF64Array(k);
    Reader.ReadF64Array(r);
    Reader.ReadF64Array(p);
    Reader.ReadU32(binning_x);
    Reader.ReadU32(binning_y);
    roi.Deserialize(Reader);
    return Reader.IsOk();
}

// ---------------------------------------------------------------------------------------------
// tf2_msgs / rosgraph_msgs

void FUBotRosTfMessage::Serialize(FUBotCdrWriter& Writer) const
{
    UBotROSMessageTypesPrivate::WriteMessageSequence(Writer, transforms);
}

bool FUBotRosTfMessage::Deserialize(FUBotCdrReader& Reader)
{
    UBotROSMessageTypesPrivate::ReadMessageSequence(Reader, transforms, UBotROSMessageTypesPrivate::MinTransformStampedBytes);
    return Reader.IsOk();
}

void FUBotRosClock::Serialize(FUBotCdrWriter& Writer) const
{
    clock.Serialize(Writer);
}

bool FUBotRosClock::Deserialize(FUBotCdrReader& Reader)
{
    clock.Deserialize(Reader);
    return Reader.IsOk();
}
