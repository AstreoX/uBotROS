#pragma once

#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "UBotROSConventionsLibrary.generated.h"

/**
 * Blueprint access to the ROS-specific conventions in UBot::ROSConventions: camera optical frame,
 * builtin_interfaces/Time fields and ROS names. The UE <-> right-handed axis mapping ROS uses is in
 * uBotCore (category uBot|Conventions, nodes "UE To Right-Handed ...").
 */
UCLASS()
class UBOTROS_API UUBotROSConventionsLibrary : public UBlueprintFunctionLibrary
{
    GENERATED_BODY()

public:
    /** Rotation of the camera optical frame (z forward, x right, y down) relative to the camera link frame, in ROS conventions. */
    UFUNCTION(BlueprintPure, Category = "uBot|ROS|Conventions", meta = (DisplayName = "Camera Link To Optical Rotation", Keywords = "ROS camera optical frame"))
    static FQuat CameraLinkToOpticalRotation();

    /** Suffix appended to a camera frame id to name its optical frame ("_optical"). */
    UFUNCTION(BlueprintPure, Category = "uBot|ROS|Conventions", meta = (DisplayName = "Get Optical Frame Suffix", Keywords = "ROS camera optical frame"))
    static FString GetOpticalFrameSuffix();

    /** Seconds -> builtin_interfaces/Time fields. Negative or non-finite input yields (0, 0). */
    UFUNCTION(BlueprintPure, Category = "uBot|ROS|Conventions", meta = (DisplayName = "Seconds To ROS Time", Keywords = "ROS stamp nanoseconds"))
    static void SecondsToRosTime(double Seconds, int32& Sec, int32& Nanosec);

    /** builtin_interfaces/Time fields -> seconds. Negative nanoseconds are treated as 0. */
    UFUNCTION(BlueprintPure, Category = "uBot|ROS|Conventions", meta = (DisplayName = "ROS Time To Seconds", Keywords = "ROS stamp nanoseconds"))
    static double RosTimeToSeconds(int32 Sec, int32 Nanosec);

    /** Makes a valid ROS name token: [A-Za-z0-9_] only, no repeated underscores, no leading digit, never empty. */
    UFUNCTION(BlueprintPure, Category = "uBot|ROS|Conventions", meta = (DisplayName = "Sanitize ROS Name", Keywords = "ROS topic namespace frame"))
    static FString SanitizeRosName(const FString& Name);
};
