#pragma once

#include "CoreMinimal.h"

/**
 * ROS-specific conventions: the camera optical frame, the builtin_interfaces/Time split and ROS name
 * tokens. Pure functions, no ROS types.
 *
 * The axis mapping between UE (left-handed, centimeters) and the right-handed FLU frame of ROS
 * (REP-103, meters) is not here: it is neutral and lives in uBotCore, UBot::RightHanded
 * (UBotRightHandedConventions.h). Everything below only means something to ROS.
 */
namespace UBot::ROSConventions
{
    /**
     * Rotation of the camera optical frame (z forward, x right, y down) relative to the camera link
     * frame, in ROS conventions: (x=-0.5, y=0.5, z=-0.5, w=0.5).
     */
    UBOTROS_API FQuat CameraLinkToOpticalRotation();

    /** Suffix appended to a camera frame id to name its optical frame: "_optical". */
    UBOTROS_API const TCHAR* OpticalFrameSuffix();

    /**
     * Seconds -> (sec, nanosec) as used by builtin_interfaces/Time. Negative or non-finite input
     * yields (0, 0). Nanoseconds are rounded and clamped to [0, 999999999]; the rounding carry goes
     * into sec. Values beyond the int32 range saturate at (MAX_int32, 999999999).
     */
    UBOTROS_API void SecondsToRosTime(double Seconds, int32& OutSec, uint32& OutNanosec);

    /** (sec, nanosec) -> seconds. */
    UBOTROS_API double RosTimeToSeconds(int32 Sec, uint32 Nanosec);

    /**
     * ROS name token: keeps [A-Za-z0-9_], replaces everything else by '_', collapses repeated
     * underscores, prefixes '_' when the first character is a digit and never returns an empty
     * string (fallback "ubot").
     */
    UBOTROS_API FString SanitizeRosName(const FString& Name);
}
