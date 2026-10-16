#include "UBotROSConventionsLibrary.h"

#include "UBotROSConventionHelpers.h"

FQuat UUBotROSConventionsLibrary::CameraLinkToOpticalRotation()
{
    return UBot::ROSConventions::CameraLinkToOpticalRotation();
}

FString UUBotROSConventionsLibrary::GetOpticalFrameSuffix()
{
    return FString(UBot::ROSConventions::OpticalFrameSuffix());
}

void UUBotROSConventionsLibrary::SecondsToRosTime(double Seconds, int32& Sec, int32& Nanosec)
{
    uint32 UnsignedNanosec = 0;
    UBot::ROSConventions::SecondsToRosTime(Seconds, Sec, UnsignedNanosec);
    // Always below 1e9, so it fits an int32.
    Nanosec = static_cast<int32>(UnsignedNanosec);
}

double UUBotROSConventionsLibrary::RosTimeToSeconds(int32 Sec, int32 Nanosec)
{
    return UBot::ROSConventions::RosTimeToSeconds(Sec, static_cast<uint32>(FMath::Max(Nanosec, 0)));
}

FString UUBotROSConventionsLibrary::SanitizeRosName(const FString& Name)
{
    return UBot::ROSConventions::SanitizeRosName(Name);
}
