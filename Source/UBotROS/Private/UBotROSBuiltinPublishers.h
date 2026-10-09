#pragma once

#include "CoreMinimal.h"
#include "UBotROSSensorPublisher.h"
#include "UBotSensorTypes.h"
#include "UBotROSBuiltinPublishers.generated.h"

class FUBotExtensionRegistry;
class UUBot2DLidarFrame;
class UUBotImageFrame;
class UUBotPointCloudFrame;

// Built-in publishers for the seven UBotSensor component types. Default topics and TopicOverrides
// keys follow the legacy AgentROSSensorSuitePublisher (single sensor of a type / several of a type):
//   Pose        pose                       / <SensorId>          geometry_msgs/msg/PoseStamped  (state QoS)
//   Odometry    odom                       / <SensorId>          nav_msgs/msg/Odometry          (state QoS)
//   IMU         imu/data      (key imu)    / <SensorId>          sensor_msgs/msg/Imu            (sensor QoS)
//   Lidar2D     scan                       / <SensorId>          sensor_msgs/msg/LaserScan      (sensor QoS)
//   Lidar3D     lidar/points  (key lidar)  / <SensorId>          sensor_msgs/msg/PointCloud2    (sensor QoS)
//   RGBCamera   camera/color/image_raw + camera_info (keys rgb_camera, rgb_camera.camera_info)
//   DepthCamera camera/depth/image_raw + camera_info (keys depth_camera, depth_camera.camera_info)
//   cameras of the same type: <SensorId>/image_raw + <SensorId>/camera_info (keys <SensorId>, <SensorId>.camera_info)

/** geometry_msgs/PoseStamped from UUBotPoseSensorComponent::OnPoseDataUpdated. */
UCLASS(Transient)
class UUBotROSPosePublisher : public UUBotROSSensorPublisher
{
    GENERATED_BODY()

public:
    virtual bool Initialize(UObject* Sensor, const FUBotROSPublisherContext& Context, FString& OutError) override;
    virtual void Shutdown() override;

private:
    UFUNCTION()
    void HandlePoseData(const FUBotPoseSensorData& Data);

    int32 TopicId = 0;
};

/**
 * nav_msgs/Odometry from UUBotOdometrySensorComponent::OnOdometryDataUpdated (odom -> base TF is published by the
 * bridge component). The sensor's FrameId (default "odom") never appears in the message and is not a physical
 * link, so no base -> sensor static transform is published for it: that would close a loop with odom -> base.
 */
UCLASS(Transient)
class UUBotROSOdometryPublisher : public UUBotROSSensorPublisher
{
    GENERATED_BODY()

public:
    virtual bool Initialize(UObject* Sensor, const FUBotROSPublisherContext& Context, FString& OutError) override;
    virtual void Shutdown() override;
    virtual bool PublishesPhysicalFrame() const override { return false; }

private:
    UFUNCTION()
    void HandleOdometryData(const FUBotOdometrySensorData& Data);

    int32 TopicId = 0;
};

/** sensor_msgs/Imu from UUBotImuSensorComponent::OnImuDataUpdated. */
UCLASS(Transient)
class UUBotROSImuPublisher : public UUBotROSSensorPublisher
{
    GENERATED_BODY()

public:
    virtual bool Initialize(UObject* Sensor, const FUBotROSPublisherContext& Context, FString& OutError) override;
    virtual void Shutdown() override;

private:
    UFUNCTION()
    void HandleImuData(const FUBotImuSensorData& Data);

    int32 TopicId = 0;
};

/** sensor_msgs/LaserScan from UUBot2DLidarSensorComponent::On2DLidarFrameReady. */
UCLASS(Transient)
class UUBotROSLidar2DPublisher : public UUBotROSSensorPublisher
{
    GENERATED_BODY()

public:
    virtual bool Initialize(UObject* Sensor, const FUBotROSPublisherContext& Context, FString& OutError) override;
    virtual void Shutdown() override;

private:
    UFUNCTION()
    void HandleScanFrame(UUBot2DLidarFrame* Frame);

    int32 TopicId = 0;
};

/** sensor_msgs/PointCloud2 (uBot layout, point_step 22) from UUBot3DLidarSensorComponent::On3DLidarFrameReady. */
UCLASS(Transient)
class UUBotROSLidar3DPublisher : public UUBotROSSensorPublisher
{
    GENERATED_BODY()

public:
    virtual bool Initialize(UObject* Sensor, const FUBotROSPublisherContext& Context, FString& OutError) override;
    virtual void Shutdown() override;

private:
    UFUNCTION()
    void HandlePointCloudFrame(UUBotPointCloudFrame* Frame);

    int32 TopicId = 0;
};

/**
 * sensor_msgs/Image + CameraInfo in the optical frame from UUBotCameraSensorComponent::OnImageFrameReady.
 * CameraInfo is published whenever it has (or may have) subscribers; the GPU readback for the image
 * only runs when the image topic does.
 */
UCLASS(Abstract, Transient)
class UUBotROSCameraPublisher : public UUBotROSSensorPublisher
{
    GENERATED_BODY()

public:
    virtual bool Initialize(UObject* Sensor, const FUBotROSPublisherContext& Context, FString& OutError) override;
    virtual void Shutdown() override;
    virtual bool UsesOpticalFrame() const override { return true; }

protected:
    /** Layout used when the camera is the only one of its type; set by the subclass constructors. */
    FString SingleSensorLogicalKey;
    FString SingleSensorImagePath;
    FString SingleSensorInfoPath;

private:
    UFUNCTION()
    void HandleImageFrame(UUBotImageFrame* Frame);

    int32 ImageTopicId = 0;
    int32 InfoTopicId = 0;
    bool bReportedReadbackFailure = false;
};

UCLASS(Transient)
class UUBotROSRgbCameraPublisher : public UUBotROSCameraPublisher
{
    GENERATED_BODY()

public:
    UUBotROSRgbCameraPublisher();
};

UCLASS(Transient)
class UUBotROSDepthCameraPublisher : public UUBotROSCameraPublisher
{
    GENERATED_BODY()

public:
    UUBotROSDepthCameraPublisher();
};

namespace UBot::ROSBuiltin
{
    /** Extension names of the built-in factories, e.g. "UBotROS.IMU", in registration order. */
    TArray<FName> GetBuiltinFactoryNames();

    /**
     * Registers the seven built-in factories (priority 0) at the sensor publisher extension point for
     * OwnerPackage. Returns how many were registered (7 unless some were already registered).
     */
    int32 RegisterBuiltinFactories(FUBotExtensionRegistry& Registry, FName OwnerPackage);
}
