#include "CoreMinimal.h"
#include "Math/RandomStream.h"
#include "Misc/AutomationTest.h"
#include "UBotROSConventionHelpers.h"
#include "UBotROSConventionsLibrary.h"
#include "UBotRightHandedConventions.h"
#include "UObject/Class.h"

#include <limits>

#if WITH_DEV_AUTOMATION_TESTS

namespace UBotROSConventionsTests
{
    constexpr int32 RandomSeed = 20251009;
    constexpr int32 RandomIterations = 500;
    constexpr double Tolerance = 1.e-9;
    constexpr EAutomationTestFlags TestFlags = EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter;

    // Random draws are sequenced into locals because argument evaluation order is unspecified, and
    // the fixed seed should give the same values with every compiler.

    /** Uniformly distributed unit quaternion, including negative w and both signs of each rotation. */
    FQuat RandomRotation(FRandomStream& Stream)
    {
        FQuat Candidate = FQuat::Identity;
        double SizeSquared = 0.0;
        do
        {
            Candidate.X = Stream.FRandRange(-1.0, 1.0);
            Candidate.Y = Stream.FRandRange(-1.0, 1.0);
            Candidate.Z = Stream.FRandRange(-1.0, 1.0);
            Candidate.W = Stream.FRandRange(-1.0, 1.0);
            SizeSquared = Candidate.SizeSquared();
        }
        while (SizeSquared > 1.0 || SizeSquared < 0.01);
        return Candidate.GetNormalized();
    }

    double VectorError(const FVector& A, const FVector& B)
    {
        return (A - B).GetAbsMax();
    }

    bool IsValidRosName(const FString& Name)
    {
        if (Name.IsEmpty() || (Name[0] >= TEXT('0') && Name[0] <= TEXT('9')))
        {
            return false;
        }
        for (int32 Index = 0; Index < Name.Len(); ++Index)
        {
            const TCHAR Character = Name[Index];
            const bool bAllowed = (Character >= TEXT('a') && Character <= TEXT('z'))
                || (Character >= TEXT('A') && Character <= TEXT('Z'))
                || (Character >= TEXT('0') && Character <= TEXT('9'))
                || Character == TEXT('_');
            if (!bAllowed || (Character == TEXT('_') && Index > 0 && Name[Index - 1] == TEXT('_')))
            {
                return false;
            }
        }
        return true;
    }

    /** Random text over an alphabet that exercises every sanitizer branch. */
    FString RandomName(FRandomStream& Stream)
    {
        const TCHAR Alphabet[] = TEXT("aZ09_ -./:~!é中");
        const int32 AlphabetLength = UE_ARRAY_COUNT(Alphabet) - 1;
        FString Input;
        const int32 Length = Stream.RandRange(0, 12);
        for (int32 Index = 0; Index < Length; ++Index)
        {
            Input.AppendChar(Alphabet[Stream.RandRange(0, AlphabetLength - 1)]);
        }
        return Input;
    }
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotROSConventionsCameraOpticalTest, "UBotROS.Conventions.CameraOpticalFrame", UBotROSConventionsTests::TestFlags)

bool FUBotROSConventionsCameraOpticalTest::RunTest(const FString& Parameters)
{
    using namespace UBotROSConventionsTests;

    const FQuat LinkToOptical = UBot::ROSConventions::CameraLinkToOpticalRotation();
    TestTrue(TEXT("Optical rotation is normalized"), FMath::Abs(LinkToOptical.Size() - 1.0) <= Tolerance);
    TestTrue(TEXT("Optical rotation components"), FMath::Abs(LinkToOptical.X + 0.5) <= Tolerance && FMath::Abs(LinkToOptical.Y - 0.5) <= Tolerance
        && FMath::Abs(LinkToOptical.Z + 0.5) <= Tolerance && FMath::Abs(LinkToOptical.W - 0.5) <= Tolerance);
    TestTrue(TEXT("Optical +Z (forward) is link +X"), VectorError(LinkToOptical.RotateVector(FVector(0.0, 0.0, 1.0)), FVector(1.0, 0.0, 0.0)) <= Tolerance);
    TestTrue(TEXT("Optical +X (right) is link -Y"), VectorError(LinkToOptical.RotateVector(FVector(1.0, 0.0, 0.0)), FVector(0.0, -1.0, 0.0)) <= Tolerance);
    TestTrue(TEXT("Optical +Y (down) is link -Z"), VectorError(LinkToOptical.RotateVector(FVector(0.0, 1.0, 0.0)), FVector(0.0, 0.0, -1.0)) <= Tolerance);

    // End to end: a UE camera's forward, right and down directions are the ROS optical +Z, +X and +Y.
    FRandomStream Stream(RandomSeed + 5);
    double AxisError = 0.0;
    for (int32 Iteration = 0; Iteration < RandomIterations; ++Iteration)
    {
        const FQuat CameraUE = RandomRotation(Stream);
        const FQuat OpticalROS = UBot::RightHanded::RotationFromUE(CameraUE) * LinkToOptical;
        AxisError = FMath::Max(AxisError, VectorError(OpticalROS.RotateVector(FVector::ZAxisVector), UBot::RightHanded::VectorFromUE(CameraUE.GetForwardVector())));
        AxisError = FMath::Max(AxisError, VectorError(OpticalROS.RotateVector(FVector::XAxisVector), UBot::RightHanded::VectorFromUE(CameraUE.GetRightVector())));
        AxisError = FMath::Max(AxisError, VectorError(OpticalROS.RotateVector(FVector::YAxisVector), UBot::RightHanded::VectorFromUE(-CameraUE.GetUpVector())));
    }
    TestTrue(*FString::Printf(TEXT("UE camera axes match the ROS optical frame (max error %g)"), AxisError), AxisError <= Tolerance);

    TestEqualSensitive(TEXT("Optical frame suffix"), UBot::ROSConventions::OpticalFrameSuffix(), TEXT("_optical"));
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotROSConventionsRosTimeTest, "UBotROS.Conventions.RosTime", UBotROSConventionsTests::TestFlags)

bool FUBotROSConventionsRosTimeTest::RunTest(const FString& Parameters)
{
    using namespace UBotROSConventionsTests;

    const auto Check = [this](double Seconds, int32 ExpectedSec, uint32 ExpectedNanosec)
    {
        int32 Sec = -1;
        uint32 Nanosec = MAX_uint32;
        UBot::ROSConventions::SecondsToRosTime(Seconds, Sec, Nanosec);
        TestEqual(*FString::Printf(TEXT("%.10f s -> sec"), Seconds), Sec, ExpectedSec);
        TestEqual(*FString::Printf(TEXT("%.10f s -> nanosec"), Seconds), static_cast<int64>(Nanosec), static_cast<int64>(ExpectedNanosec));
    };

    Check(0.0, 0, 0);
    Check(-1.0, 0, 0);
    Check(-1.e-12, 0, 0);
    Check(std::numeric_limits<double>::quiet_NaN(), 0, 0);
    Check(std::numeric_limits<double>::infinity(), 0, 0);
    Check(-std::numeric_limits<double>::infinity(), 0, 0);
    Check(0.9999999999, 1, 0);
    Check(41.9999999996, 42, 0);
    Check(1.5, 1, 500000000);
    Check(1.25, 1, 250000000);
    Check(2.000000001, 2, 1);
    Check(12345.678, 12345, 678000000);
    Check(2147483647.25, MAX_int32, 250000000);
    Check(3.0e9, MAX_int32, 999999999);

    for (int32 WholeSeconds = 1; WholeSeconds <= 100; ++WholeSeconds)
    {
        int32 Sec = 0;
        uint32 Nanosec = 0;
        UBot::ROSConventions::SecondsToRosTime(static_cast<double>(WholeSeconds) - 1.e-11, Sec, Nanosec);
        if (Sec != WholeSeconds || Nanosec != 0)
        {
            AddError(FString::Printf(TEXT("%d s - 10 ps should carry to (%d, 0), got (%d, %u)"), WholeSeconds, WholeSeconds, Sec, Nanosec));
            break;
        }
    }

    TestEqual(TEXT("ROS time to seconds"), UBot::ROSConventions::RosTimeToSeconds(1, 500000000), 1.5, 0.0);
    TestEqual(TEXT("ROS time zero"), UBot::ROSConventions::RosTimeToSeconds(0, 0), 0.0, 0.0);
    TestEqual(TEXT("ROS time quarter"), UBot::ROSConventions::RosTimeToSeconds(12, 250000000), 12.25, 0.0);

    FRandomStream Stream(RandomSeed + 6);
    int32 RoundTripFailures = 0;
    double MaxQuantizationError = 0.0;
    bool bNanosecondsInRange = true;
    for (int32 Iteration = 0; Iteration < 2000; ++Iteration)
    {
        const int32 Sec = Stream.RandRange(0, 1000000);
        const uint32 Milliseconds = static_cast<uint32>(Stream.RandRange(0, 999));
        const uint32 Nanosec = Milliseconds * 1000000u + static_cast<uint32>(Stream.RandRange(0, 999999));
        int32 OutSec = 0;
        uint32 OutNanosec = 0;
        UBot::ROSConventions::SecondsToRosTime(UBot::ROSConventions::RosTimeToSeconds(Sec, Nanosec), OutSec, OutNanosec);
        RoundTripFailures += (OutSec != Sec || OutNanosec != Nanosec) ? 1 : 0;

        const double Seconds = Stream.FRandRange(0.0, 1000000.0);
        UBot::ROSConventions::SecondsToRosTime(Seconds, OutSec, OutNanosec);
        bNanosecondsInRange = bNanosecondsInRange && OutNanosec <= 999999999u;
        MaxQuantizationError = FMath::Max(MaxQuantizationError, FMath::Abs(UBot::ROSConventions::RosTimeToSeconds(OutSec, OutNanosec) - Seconds));
    }
    TestEqual(TEXT("(sec, nanosec) -> seconds -> (sec, nanosec) round trip failures"), RoundTripFailures, 0);
    TestTrue(TEXT("Nanoseconds stay within [0, 999999999]"), bNanosecondsInRange);
    TestTrue(*FString::Printf(TEXT("Conversion error is at most half a nanosecond (max %g s)"), MaxQuantizationError), MaxQuantizationError <= 0.6e-9);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotROSConventionsSanitizeTest, "UBotROS.Conventions.SanitizeRosName", UBotROSConventionsTests::TestFlags)

bool FUBotROSConventionsSanitizeTest::RunTest(const FString& Parameters)
{
    using namespace UBotROSConventionsTests;

    struct FCase
    {
        const TCHAR* Input;
        const TCHAR* Expected;
    };
    const FCase Cases[] =
    {
        { TEXT(""), TEXT("ubot") },
        { TEXT("robot_1"), TEXT("robot_1") },
        { TEXT("BaseLink"), TEXT("BaseLink") },
        { TEXT("Robot A"), TEXT("Robot_A") },
        { TEXT("Rover-01"), TEXT("Rover_01") },
        { TEXT("/fleet/rover_a/"), TEXT("_fleet_rover_a_") },
        { TEXT("a--b"), TEXT("a_b") },
        { TEXT("a__b"), TEXT("a_b") },
        { TEXT("a - b"), TEXT("a_b") },
        { TEXT("robot.v2"), TEXT("robot_v2") },
        { TEXT("1robot"), TEXT("_1robot") },
        { TEXT("9"), TEXT("_9") },
        { TEXT("-1robot"), TEXT("_1robot") },
        { TEXT("_1robot"), TEXT("_1robot") },
        { TEXT("roböt"), TEXT("rob_t") },
        { TEXT("机器人_7"), TEXT("_7") },
        { TEXT("!!!"), TEXT("_") },
    };
    for (const FCase& Case : Cases)
    {
        TestEqualSensitive(*FString::Printf(TEXT("SanitizeRosName(\"%s\")"), Case.Input), UBot::ROSConventions::SanitizeRosName(Case.Input), Case.Expected);
    }

    // Every output must be a valid ROS name token, whatever the input.
    FRandomStream Stream(RandomSeed + 7);
    int32 InvalidCount = 0;
    for (int32 Iteration = 0; Iteration < RandomIterations; ++Iteration)
    {
        const FString Input = RandomName(Stream);
        const FString Output = UBot::ROSConventions::SanitizeRosName(Input);
        if (!IsValidRosName(Output))
        {
            ++InvalidCount;
            AddError(FString::Printf(TEXT("SanitizeRosName(\"%s\") produced invalid name \"%s\""), *Input, *Output));
        }
    }
    TestEqual(TEXT("Invalid sanitized names"), InvalidCount, 0);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotROSConventionsBlueprintLibraryTest, "UBotROS.Conventions.BlueprintLibrary", UBotROSConventionsTests::TestFlags)

bool FUBotROSConventionsBlueprintLibraryTest::RunTest(const FString& Parameters)
{
    using namespace UBotROSConventionsTests;

    // Optical frame.
    TestTrue(TEXT("Blueprint optical rotation is the C++ one"), UUBotROSConventionsLibrary::CameraLinkToOpticalRotation() == UBot::ROSConventions::CameraLinkToOpticalRotation());
    TestEqualSensitive(TEXT("Blueprint optical frame suffix"), UUBotROSConventionsLibrary::GetOpticalFrameSuffix(), TEXT("_optical"));

    // Seconds -> ROS time, including the carry, saturation and invalid input.
    int32 BlueprintSec = -1;
    int32 BlueprintNanosec = -1;
    UUBotROSConventionsLibrary::SecondsToRosTime(0.9999999999, BlueprintSec, BlueprintNanosec);
    TestTrue(TEXT("Blueprint seconds to ROS time carries"), BlueprintSec == 1 && BlueprintNanosec == 0);
    UUBotROSConventionsLibrary::SecondsToRosTime(7.125, BlueprintSec, BlueprintNanosec);
    TestTrue(TEXT("Blueprint seconds to ROS time"), BlueprintSec == 7 && BlueprintNanosec == 125000000);
    UUBotROSConventionsLibrary::SecondsToRosTime(3.0e9, BlueprintSec, BlueprintNanosec);
    TestTrue(TEXT("Blueprint seconds to ROS time saturates"), BlueprintSec == MAX_int32 && BlueprintNanosec == 999999999);
    UUBotROSConventionsLibrary::SecondsToRosTime(std::numeric_limits<double>::quiet_NaN(), BlueprintSec, BlueprintNanosec);
    TestTrue(TEXT("Blueprint seconds to ROS time: NaN is (0, 0)"), BlueprintSec == 0 && BlueprintNanosec == 0);
    UUBotROSConventionsLibrary::SecondsToRosTime(-2.5, BlueprintSec, BlueprintNanosec);
    TestTrue(TEXT("Blueprint seconds to ROS time: negative is (0, 0)"), BlueprintSec == 0 && BlueprintNanosec == 0);

    // ROS time -> seconds; the signed nanoseconds of a Blueprint pin cannot go below zero.
    TestEqual(TEXT("Blueprint ROS time to seconds"), UUBotROSConventionsLibrary::RosTimeToSeconds(7, 125000000), 7.125, 0.0);
    TestEqual(TEXT("Blueprint negative nanoseconds count as zero"), UUBotROSConventionsLibrary::RosTimeToSeconds(7, -5), 7.0, 0.0);
    TestEqual(TEXT("Blueprint minimum nanoseconds count as zero"), UUBotROSConventionsLibrary::RosTimeToSeconds(7, MIN_int32), 7.0, 0.0);

    // The nodes give exactly what the C++ functions give.
    FRandomStream Stream(RandomSeed + 8);
    int32 TimeMismatches = 0;
    int32 ToSecondsMismatches = 0;
    int32 NameMismatches = 0;
    for (int32 Iteration = 0; Iteration < RandomIterations; ++Iteration)
    {
        const double Seconds = Stream.FRandRange(-10.0, 1000000.0);
        int32 Sec = 0;
        uint32 Nanosec = 0;
        UBot::ROSConventions::SecondsToRosTime(Seconds, Sec, Nanosec);
        UUBotROSConventionsLibrary::SecondsToRosTime(Seconds, BlueprintSec, BlueprintNanosec);
        TimeMismatches += (BlueprintSec != Sec || BlueprintNanosec != static_cast<int32>(Nanosec)) ? 1 : 0;

        const int32 InputSec = Stream.RandRange(0, 1000000);
        const int32 InputNanosec = Stream.RandRange(-1000, 999999999);
        const double Expected = UBot::ROSConventions::RosTimeToSeconds(InputSec, static_cast<uint32>(FMath::Max(InputNanosec, 0)));
        ToSecondsMismatches += (UUBotROSConventionsLibrary::RosTimeToSeconds(InputSec, InputNanosec) != Expected) ? 1 : 0;

        const FString Name = RandomName(Stream);
        NameMismatches += (UUBotROSConventionsLibrary::SanitizeRosName(Name) != UBot::ROSConventions::SanitizeRosName(Name)) ? 1 : 0;
    }
    TestEqual(TEXT("Seconds to ROS time node differs from the C++ function"), TimeMismatches, 0);
    TestEqual(TEXT("ROS time to seconds node differs from the C++ function"), ToSecondsMismatches, 0);
    TestEqual(TEXT("Sanitize node differs from the C++ function"), NameMismatches, 0);
    TestEqualSensitive(TEXT("Blueprint sanitize"), UUBotROSConventionsLibrary::SanitizeRosName(TEXT("Robot A")), TEXT("Robot_A"));
    TestEqualSensitive(TEXT("Blueprint sanitize of an empty name"), UUBotROSConventionsLibrary::SanitizeRosName(FString()), TEXT("ubot"));

    // The nodes are exposed as pure static functions in uBot|ROS|Conventions, none of them deprecated.
    struct FNode
    {
        const TCHAR* Function;
        const TCHAR* DisplayName;
        const TCHAR* Keywords;
    };
    const FNode Nodes[] =
    {
        { TEXT("CameraLinkToOpticalRotation"), TEXT("Camera Link To Optical Rotation"), TEXT("ROS camera optical frame") },
        { TEXT("GetOpticalFrameSuffix"), TEXT("Get Optical Frame Suffix"), TEXT("ROS camera optical frame") },
        { TEXT("SecondsToRosTime"), TEXT("Seconds To ROS Time"), TEXT("ROS stamp nanoseconds") },
        { TEXT("RosTimeToSeconds"), TEXT("ROS Time To Seconds"), TEXT("ROS stamp nanoseconds") },
        { TEXT("SanitizeRosName"), TEXT("Sanitize ROS Name"), TEXT("ROS topic namespace frame") },
    };
    for (const FNode& Node : Nodes)
    {
        const UFunction* Function = UUBotROSConventionsLibrary::StaticClass()->FindFunctionByName(FName(Node.Function));
        if (!TestNotNull(*FString::Printf(TEXT("Function %s exists"), Node.Function), Function))
        {
            continue;
        }
        TestTrue(*FString::Printf(TEXT("%s is static"), Node.Function), Function->HasAnyFunctionFlags(FUNC_Static));
        TestTrue(*FString::Printf(TEXT("%s is Blueprint callable"), Node.Function), Function->HasAnyFunctionFlags(FUNC_BlueprintCallable));
        TestTrue(*FString::Printf(TEXT("%s is pure"), Node.Function), Function->HasAnyFunctionFlags(FUNC_BlueprintPure));
#if WITH_METADATA
        TestEqualSensitive(*FString::Printf(TEXT("%s category"), Node.Function), Function->GetMetaData(TEXT("Category")), TEXT("uBot|ROS|Conventions"));
        TestEqualSensitive(*FString::Printf(TEXT("%s display name"), Node.Function), Function->GetMetaData(TEXT("DisplayName")), Node.DisplayName);
        TestEqualSensitive(*FString::Printf(TEXT("%s keywords"), Node.Function), Function->GetMetaData(TEXT("Keywords")), Node.Keywords);
        TestFalse(*FString::Printf(TEXT("%s is not deprecated"), Node.Function), Function->HasMetaData(TEXT("DeprecatedFunction")));
#endif
    }
    return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
