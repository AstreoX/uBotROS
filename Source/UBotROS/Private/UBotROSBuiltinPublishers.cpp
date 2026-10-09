#include "UBotROSBuiltinPublishers.h"

#include "UBot2DLidarFrame.h"
#include "UBot2DLidarSensorComponent.h"
#include "UBot3DLidarSensorComponent.h"
#include "UBotCameraSensorComponent.h"
#include "UBotDepthCameraSensorComponent.h"
#include "UBotExtensionRegistry.h"
#include "UBotImageFrame.h"
#include "UBotImuSensorComponent.h"
#include "UBotOdometrySensorComponent.h"
#include "UBotPointCloudFrame.h"
#include "UBotPoseSensorComponent.h"
#include "UBotRgbCameraSensorComponent.h"
#include "UBotROS.h"
#include "UBotROSConverters.h"
#include "UBotROSMessageTypes.h"
#include "UBotROSSensorPublisher.h"
#include "UObject/Package.h"

namespace UBotROSBuiltinPublishersPrivate
{
    /** Logical key and default path of a non-camera publisher (legacy suite publisher layout). */
    struct FTopicLayout
    {
        FString LogicalKey;
        FString DefaultPath;
    };

    FTopicLayout MakeLayout(bool bSoleSensorOfType, const FString& SensorToken, const TCHAR* SingleKey, const TCHAR* SinglePath)
    {
        FTopicLayout Layout;
        if (bSoleSensorOfType)
        {
            Layout.LogicalKey = SingleKey;
            Layout.DefaultPath = SinglePath;
        }
        else
        {
            Layout.LogicalKey = SensorToken;
            Layout.DefaultPath = SensorToken;
        }
        return Layout;
    }

    /** Frame id through the context, or the resolved fallback frame when RawFrameId is empty. */
    template <typename TResolve>
    FString ResolveFrameOr(const TResolve& Resolve, const FString& RawFrameId, const TCHAR* Fallback)
    {
        FString Frame = Resolve(RawFrameId);
        if (Frame.IsEmpty())
        {
            Frame = Resolve(FString(Fallback));
        }
        return Frame;
    }

    template <typename TSensor, typename TPublisher>
    class TUBotROSBuiltinPublisherFactory final : public IUBotROSSensorPublisherFactory
    {
    public:
        explicit TUBotROSBuiltinPublisherFactory(FName InName)
            : Name(InName)
        {
        }

        virtual FName GetExtensionName() const override
        {
            return Name;
        }

        virtual bool SupportsSensor(const UObject* Sensor) const override
        {
            return Sensor && Sensor->IsA<TSensor>();
        }

        virtual UUBotROSSensorPublisher* CreatePublisher(UObject* Outer, UObject* Sensor) const override
        {
            if (!SupportsSensor(Sensor))
            {
                return nullptr;
            }
            return NewObject<TPublisher>(Outer ? Outer : GetTransientPackage(), NAME_None, RF_Transient);
        }

    private:
        FName Name;
    };

    const TCHAR* const PoseFactoryName = TEXT("UBotROS.Pose");
    const TCHAR* const OdometryFactoryName = TEXT("UBotROS.Odometry");
    const TCHAR* const ImuFactoryName = TEXT("UBotROS.IMU");
    const TCHAR* const Lidar2DFactoryName = TEXT("UBotROS.Lidar2D");
    const TCHAR* const Lidar3DFactoryName = TEXT("UBotROS.Lidar3D");
    const TCHAR* const RgbCameraFactoryName = TEXT("UBotROS.RGBCamera");
    const TCHAR* const DepthCameraFactoryName = TEXT("UBotROS.DepthCamera");
}

// ---------------------------------------------------------------------------------------------------
// Pose

bool UUBotROSPosePublisher::Initialize(UObject* Sensor, const FUBotROSPublisherContext& InContext, FString& OutError)
{
    if (!Super::Initialize(Sensor, InContext, OutError))
    {
        return false;
    }
    UUBotPoseSensorComponent* PoseSensor = Cast<UUBotPoseSensorComponent>(Sensor);
    if (!PoseSensor)
    {
        OutError = FString::Printf(TEXT("Sensor '%s' is not a uBot pose sensor."), *GetSensorId());
        Shutdown();
        return false;
    }

    const UBotROSBuiltinPublishersPrivate::FTopicLayout Layout = UBotROSBuiltinPublishersPrivate::MakeLayout(IsSoleSensorOfType(), GetSensorToken(), TEXT("pose"), TEXT("pose"));
    TopicId = AdvertiseTopic(Layout.LogicalKey, Layout.DefaultPath, FUBotRosPoseStamped::TypeName(), GetContext().StateQos, OutError);
    if (TopicId <= 0)
    {
        Shutdown();
        return false;
    }

    PoseSensor->OnPoseDataUpdated.AddUniqueDynamic(this, &UUBotROSPosePublisher::HandlePoseData);
    return true;
}

void UUBotROSPosePublisher::Shutdown()
{
    if (UUBotPoseSensorComponent* PoseSensor = Cast<UUBotPoseSensorComponent>(GetSensor()))
    {
        PoseSensor->OnPoseDataUpdated.RemoveDynamic(this, &UUBotROSPosePublisher::HandlePoseData);
    }
    TopicId = 0;
    Super::Shutdown();
}

void UUBotROSPosePublisher::HandlePoseData(const FUBotPoseSensorData& Data)
{
    if (!ShouldPublishSample(Data.Header) || !WantsMessage(TopicId))
    {
        return;
    }

    // A world-referenced pose is expressed in a frame shared by every robot, so it is never prefixed
    // with the robot namespace; parent/custom reference frames belong to the robot and are resolved.
    const UUBotPoseSensorComponent* PoseSensor = Cast<UUBotPoseSensorComponent>(GetSensor());
    const bool bWorldReferenced = !PoseSensor || PoseSensor->ReferenceFrame == EUBotPoseReferenceFrame::World;
    FString FrameId = bWorldReferenced ? UBot::ROSPublishing::SanitizeRosPath(Data.ReferenceFrameId) : ResolveFrameId(Data.ReferenceFrameId);
    if (FrameId.IsEmpty())
    {
        FrameId = TEXT("world");
    }

    PublishMessage(TopicId, UBot::ROSConvert::MakePose(Data, FrameId));
}

// ---------------------------------------------------------------------------------------------------
// Odometry

bool UUBotROSOdometryPublisher::Initialize(UObject* Sensor, const FUBotROSPublisherContext& InContext, FString& OutError)
{
    if (!Super::Initialize(Sensor, InContext, OutError))
    {
        return false;
    }
    UUBotOdometrySensorComponent* OdometrySensor = Cast<UUBotOdometrySensorComponent>(Sensor);
    if (!OdometrySensor)
    {
        OutError = FString::Printf(TEXT("Sensor '%s' is not a uBot odometry sensor."), *GetSensorId());
        Shutdown();
        return false;
    }

    const UBotROSBuiltinPublishersPrivate::FTopicLayout Layout = UBotROSBuiltinPublishersPrivate::MakeLayout(IsSoleSensorOfType(), GetSensorToken(), TEXT("odom"), TEXT("odom"));
    TopicId = AdvertiseTopic(Layout.LogicalKey, Layout.DefaultPath, FUBotRosOdometry::TypeName(), GetContext().StateQos, OutError);
    if (TopicId <= 0)
    {
        Shutdown();
        return false;
    }

    OdometrySensor->OnOdometryDataUpdated.AddUniqueDynamic(this, &UUBotROSOdometryPublisher::HandleOdometryData);
    return true;
}

void UUBotROSOdometryPublisher::Shutdown()
{
    if (UUBotOdometrySensorComponent* OdometrySensor = Cast<UUBotOdometrySensorComponent>(GetSensor()))
    {
        OdometrySensor->OnOdometryDataUpdated.RemoveDynamic(this, &UUBotROSOdometryPublisher::HandleOdometryData);
    }
    TopicId = 0;
    Super::Shutdown();
}

void UUBotROSOdometryPublisher::HandleOdometryData(const FUBotOdometrySensorData& Data)
{
    if (!ShouldPublishSample(Data.Header) || !WantsMessage(TopicId))
    {
        return;
    }

    auto Resolve = [this](const FString& RawFrameId)
    {
        return ResolveFrameId(RawFrameId);
    };
    const FString OdometryFrameId = UBotROSBuiltinPublishersPrivate::ResolveFrameOr(Resolve, Data.OdometryFrameId, TEXT("odom"));
    const FString ChildFrameId = UBotROSBuiltinPublishersPrivate::ResolveFrameOr(Resolve, Data.ChildFrameId, TEXT("base_link"));
    PublishMessage(TopicId, UBot::ROSConvert::MakeOdometry(Data, OdometryFrameId, ChildFrameId));
}

// ---------------------------------------------------------------------------------------------------
// IMU

bool UUBotROSImuPublisher::Initialize(UObject* Sensor, const FUBotROSPublisherContext& InContext, FString& OutError)
{
    if (!Super::Initialize(Sensor, InContext, OutError))
    {
        return false;
    }
    UUBotImuSensorComponent* ImuSensor = Cast<UUBotImuSensorComponent>(Sensor);
    if (!ImuSensor)
    {
        OutError = FString::Printf(TEXT("Sensor '%s' is not a uBot IMU sensor."), *GetSensorId());
        Shutdown();
        return false;
    }

    const UBotROSBuiltinPublishersPrivate::FTopicLayout Layout = UBotROSBuiltinPublishersPrivate::MakeLayout(IsSoleSensorOfType(), GetSensorToken(), TEXT("imu"), TEXT("imu/data"));
    TopicId = AdvertiseTopic(Layout.LogicalKey, Layout.DefaultPath, FUBotRosImu::TypeName(), GetContext().SensorQos, OutError);
    if (TopicId <= 0)
    {
        Shutdown();
        return false;
    }

    ImuSensor->OnImuDataUpdated.AddUniqueDynamic(this, &UUBotROSImuPublisher::HandleImuData);
    return true;
}

void UUBotROSImuPublisher::Shutdown()
{
    if (UUBotImuSensorComponent* ImuSensor = Cast<UUBotImuSensorComponent>(GetSensor()))
    {
        ImuSensor->OnImuDataUpdated.RemoveDynamic(this, &UUBotROSImuPublisher::HandleImuData);
    }
    TopicId = 0;
    Super::Shutdown();
}

void UUBotROSImuPublisher::HandleImuData(const FUBotImuSensorData& Data)
{
    if (!ShouldPublishSample(Data.Header) || !WantsMessage(TopicId))
    {
        return;
    }
    PublishMessage(TopicId, UBot::ROSConvert::MakeImu(Data, GetSensorFrameId()));
}

// ---------------------------------------------------------------------------------------------------
// 2D LiDAR

bool UUBotROSLidar2DPublisher::Initialize(UObject* Sensor, const FUBotROSPublisherContext& InContext, FString& OutError)
{
    if (!Super::Initialize(Sensor, InContext, OutError))
    {
        return false;
    }
    UUBot2DLidarSensorComponent* LidarSensor = Cast<UUBot2DLidarSensorComponent>(Sensor);
    if (!LidarSensor)
    {
        OutError = FString::Printf(TEXT("Sensor '%s' is not a uBot 2D LiDAR sensor."), *GetSensorId());
        Shutdown();
        return false;
    }

    const UBotROSBuiltinPublishersPrivate::FTopicLayout Layout = UBotROSBuiltinPublishersPrivate::MakeLayout(IsSoleSensorOfType(), GetSensorToken(), TEXT("scan"), TEXT("scan"));
    TopicId = AdvertiseTopic(Layout.LogicalKey, Layout.DefaultPath, FUBotRosLaserScan::TypeName(), GetContext().SensorQos, OutError);
    if (TopicId <= 0)
    {
        Shutdown();
        return false;
    }

    LidarSensor->On2DLidarFrameReady.AddUniqueDynamic(this, &UUBotROSLidar2DPublisher::HandleScanFrame);
    return true;
}

void UUBotROSLidar2DPublisher::Shutdown()
{
    if (UUBot2DLidarSensorComponent* LidarSensor = Cast<UUBot2DLidarSensorComponent>(GetSensor()))
    {
        LidarSensor->On2DLidarFrameReady.RemoveDynamic(this, &UUBotROSLidar2DPublisher::HandleScanFrame);
    }
    TopicId = 0;
    Super::Shutdown();
}

void UUBotROSLidar2DPublisher::HandleScanFrame(UUBot2DLidarFrame* Frame)
{
    if (!Frame || !ShouldPublishSample(Frame->Header) || !WantsMessage(TopicId))
    {
        return;
    }
    PublishMessage(TopicId, UBot::ROSConvert::MakeLaserScan(*Frame, GetSensorFrameId()));
}

// ---------------------------------------------------------------------------------------------------
// 3D LiDAR

bool UUBotROSLidar3DPublisher::Initialize(UObject* Sensor, const FUBotROSPublisherContext& InContext, FString& OutError)
{
    if (!Super::Initialize(Sensor, InContext, OutError))
    {
        return false;
    }
    UUBot3DLidarSensorComponent* LidarSensor = Cast<UUBot3DLidarSensorComponent>(Sensor);
    if (!LidarSensor)
    {
        OutError = FString::Printf(TEXT("Sensor '%s' is not a uBot 3D LiDAR sensor."), *GetSensorId());
        Shutdown();
        return false;
    }

    const UBotROSBuiltinPublishersPrivate::FTopicLayout Layout = UBotROSBuiltinPublishersPrivate::MakeLayout(IsSoleSensorOfType(), GetSensorToken(), TEXT("lidar"), TEXT("lidar/points"));
    TopicId = AdvertiseTopic(Layout.LogicalKey, Layout.DefaultPath, FUBotRosPointCloud2::TypeName(), GetContext().SensorQos, OutError);
    if (TopicId <= 0)
    {
        Shutdown();
        return false;
    }

    LidarSensor->On3DLidarFrameReady.AddUniqueDynamic(this, &UUBotROSLidar3DPublisher::HandlePointCloudFrame);
    return true;
}

void UUBotROSLidar3DPublisher::Shutdown()
{
    if (UUBot3DLidarSensorComponent* LidarSensor = Cast<UUBot3DLidarSensorComponent>(GetSensor()))
    {
        LidarSensor->On3DLidarFrameReady.RemoveDynamic(this, &UUBotROSLidar3DPublisher::HandlePointCloudFrame);
    }
    TopicId = 0;
    Super::Shutdown();
}

void UUBotROSLidar3DPublisher::HandlePointCloudFrame(UUBotPointCloudFrame* Frame)
{
    if (!Frame || !ShouldPublishSample(Frame->Header) || !WantsMessage(TopicId))
    {
        return;
    }
    PublishMessage(TopicId, UBot::ROSConvert::MakePointCloud2(*Frame, GetSensorFrameId()));
}

// ---------------------------------------------------------------------------------------------------
// Cameras

UUBotROSRgbCameraPublisher::UUBotROSRgbCameraPublisher()
{
    SingleSensorLogicalKey = TEXT("rgb_camera");
    SingleSensorImagePath = TEXT("camera/color/image_raw");
    SingleSensorInfoPath = TEXT("camera/color/camera_info");
}

UUBotROSDepthCameraPublisher::UUBotROSDepthCameraPublisher()
{
    SingleSensorLogicalKey = TEXT("depth_camera");
    SingleSensorImagePath = TEXT("camera/depth/image_raw");
    SingleSensorInfoPath = TEXT("camera/depth/camera_info");
}

bool UUBotROSCameraPublisher::Initialize(UObject* Sensor, const FUBotROSPublisherContext& InContext, FString& OutError)
{
    if (!Super::Initialize(Sensor, InContext, OutError))
    {
        return false;
    }
    UUBotCameraSensorComponent* Camera = Cast<UUBotCameraSensorComponent>(Sensor);
    if (!Camera)
    {
        OutError = FString::Printf(TEXT("Sensor '%s' is not a uBot camera sensor."), *GetSensorId());
        Shutdown();
        return false;
    }

    FString ImageKey;
    FString ImagePath;
    FString InfoKey;
    FString InfoPath;
    if (IsSoleSensorOfType())
    {
        ImageKey = SingleSensorLogicalKey;
        ImagePath = SingleSensorImagePath;
        InfoKey = SingleSensorLogicalKey + TEXT(".camera_info");
        InfoPath = SingleSensorInfoPath;
    }
    else
    {
        const FString Token = GetSensorToken();
        ImageKey = Token;
        ImagePath = Token + TEXT("/image_raw");
        InfoKey = Token + TEXT(".camera_info");
        InfoPath = Token + TEXT("/camera_info");
    }

    ImageTopicId = AdvertiseTopic(ImageKey, ImagePath, FUBotRosImage::TypeName(), GetContext().SensorQos, OutError);
    if (ImageTopicId <= 0)
    {
        Shutdown();
        return false;
    }

    // Legacy CameraInfo QoS: reliable / volatile / depth 1, independent of the sensor QoS override.
    FUBotROSQos InfoQos = FUBotROSQos::State();
    InfoQos.Depth = 1;
    InfoTopicId = AdvertiseTopic(InfoKey, InfoPath, FUBotRosCameraInfo::TypeName(), InfoQos, OutError);
    if (InfoTopicId <= 0)
    {
        Shutdown();
        return false;
    }

    bReportedReadbackFailure = false;
    Camera->OnImageFrameReady.AddUniqueDynamic(this, &UUBotROSCameraPublisher::HandleImageFrame);
    return true;
}

void UUBotROSCameraPublisher::Shutdown()
{
    if (UUBotCameraSensorComponent* Camera = Cast<UUBotCameraSensorComponent>(GetSensor()))
    {
        Camera->OnImageFrameReady.RemoveDynamic(this, &UUBotROSCameraPublisher::HandleImageFrame);
    }
    ImageTopicId = 0;
    InfoTopicId = 0;
    Super::Shutdown();
}

void UUBotROSCameraPublisher::HandleImageFrame(UUBotImageFrame* Frame)
{
    if (!Frame || !ShouldPublishSample(Frame->Header))
    {
        return;
    }

    const FString OpticalFrameId = GetOpticalFrameId();
    if (WantsMessage(InfoTopicId))
    {
        PublishMessage(InfoTopicId, UBot::ROSConvert::MakeCameraInfo(*Frame, OpticalFrameId));
    }

    // The CPU readback flushes rendering, so it only runs when the image is actually wanted.
    if (!WantsMessage(ImageTopicId))
    {
        return;
    }
    FString ErrorMessage;
    if (!Frame->HasCPUPixelData() && !Frame->RequestPixelReadback(ErrorMessage))
    {
        if (!bReportedReadbackFailure)
        {
            bReportedReadbackFailure = true;
            UE_LOG(LogUBotROS, Warning, TEXT("Image readback failed for camera '%s': %s (further failures are logged at Verbose)."), *GetSensorId(), *ErrorMessage);
        }
        else
        {
            UE_LOG(LogUBotROS, Verbose, TEXT("Image readback failed for camera '%s': %s"), *GetSensorId(), *ErrorMessage);
        }
        return;
    }
    PublishMessage(ImageTopicId, UBot::ROSConvert::MakeImage(*Frame, OpticalFrameId));
}

// ---------------------------------------------------------------------------------------------------
// Factories

namespace UBot::ROSBuiltin
{
    TArray<FName> GetBuiltinFactoryNames()
    {
        using namespace UBotROSBuiltinPublishersPrivate;
        return {
            FName(PoseFactoryName),
            FName(OdometryFactoryName),
            FName(ImuFactoryName),
            FName(Lidar2DFactoryName),
            FName(Lidar3DFactoryName),
            FName(RgbCameraFactoryName),
            FName(DepthCameraFactoryName),
        };
    }

    int32 RegisterBuiltinFactories(FUBotExtensionRegistry& Registry, FName OwnerPackage)
    {
        using namespace UBotROSBuiltinPublishersPrivate;
        const TArray<TSharedRef<IUBotExtension>> Factories =
        {
            MakeShared<TUBotROSBuiltinPublisherFactory<UUBotPoseSensorComponent, UUBotROSPosePublisher>>(FName(PoseFactoryName)),
            MakeShared<TUBotROSBuiltinPublisherFactory<UUBotOdometrySensorComponent, UUBotROSOdometryPublisher>>(FName(OdometryFactoryName)),
            MakeShared<TUBotROSBuiltinPublisherFactory<UUBotImuSensorComponent, UUBotROSImuPublisher>>(FName(ImuFactoryName)),
            MakeShared<TUBotROSBuiltinPublisherFactory<UUBot2DLidarSensorComponent, UUBotROSLidar2DPublisher>>(FName(Lidar2DFactoryName)),
            MakeShared<TUBotROSBuiltinPublisherFactory<UUBot3DLidarSensorComponent, UUBotROSLidar3DPublisher>>(FName(Lidar3DFactoryName)),
            MakeShared<TUBotROSBuiltinPublisherFactory<UUBotRgbCameraSensorComponent, UUBotROSRgbCameraPublisher>>(FName(RgbCameraFactoryName)),
            MakeShared<TUBotROSBuiltinPublisherFactory<UUBotDepthCameraSensorComponent, UUBotROSDepthCameraPublisher>>(FName(DepthCameraFactoryName)),
        };

        const FName PointName = UBot::ROSPublishing::GetExtensionPointName();
        int32 Registered = 0;
        for (const TSharedRef<IUBotExtension>& Factory : Factories)
        {
            if (Registry.RegisterExtension(PointName, OwnerPackage, Factory))
            {
                ++Registered;
            }
            else
            {
                UE_LOG(LogUBotROS, Warning, TEXT("Built-in ROS sensor publisher factory '%s' could not be registered (already registered?)."), *Factory->GetExtensionName().ToString());
            }
        }
        return Registered;
    }
}
