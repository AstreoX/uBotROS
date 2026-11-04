#include "CoreMinimal.h"
#include "Engine/MemberReference.h"
#include "Misc/AutomationTest.h"
#include "UBotConventionsLibrary.h"
#include "UBotROSConventionsLibrary.h"
#include "UObject/CoreRedirects.h"
#include "UObject/StructOnScope.h"
#include "UObject/UnrealType.h"

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotROSConventionsCoreRedirectsTest, "UBotROS.Conventions.CoreRedirects", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotROSConventionsCoreRedirectsTest::RunTest(const FString& Parameters)
{
    const FName FunctionNames[] =
    {
        TEXT("CameraLinkToOpticalRotation"),
        TEXT("GetOpticalFrameSuffix"),
        TEXT("SecondsToRosTime"),
        TEXT("RosTimeToSeconds"),
        TEXT("SanitizeRosName"),
    };
    UClass* OldLibrary = UUBotConventionsLibrary::StaticClass();
    UClass* NewLibrary = UUBotROSConventionsLibrary::StaticClass();
    TMap<FName, UFunction*> Functions;
    for (const FName FunctionName : FunctionNames)
    {
        const FString OldPath = OldLibrary->GetPathName() + TEXT(".") + FunctionName.ToString();
        const FString NewPath = NewLibrary->GetPathName() + TEXT(".") + FunctionName.ToString();
        const FCoreRedirectObjectName Redirected = FCoreRedirects::GetRedirectedName(
            ECoreRedirectFlags::Type_Function, FCoreRedirectObjectName(OldPath));
        TestEqual(*FString::Printf(TEXT("%s has a cross-plugin function redirect"), *FunctionName.ToString()),
            Redirected.ToString(), FCoreRedirectObjectName(NewPath).ToString());
        TestNull(*FString::Printf(TEXT("%s is absent from Core"), *FunctionName.ToString()), OldLibrary->FindFunctionByName(FunctionName));

        // Blueprint call nodes persist an FMemberReference, not just a function path. Exercise its
        // real redirect resolution, including updating the owning class after the move to ROS.
        FMemberReference Reference;
        Reference.SetExternalMember(FunctionName, OldLibrary);
        UFunction* Function = Reference.ResolveMember<UFunction>(nullptr, true);
        if (TestNotNull(*FString::Printf(TEXT("Legacy Blueprint reference %s resolves"), *FunctionName.ToString()), Function))
        {
            TestTrue(TEXT("The resolved function is owned by ROS"), Function->GetOwnerClass() == NewLibrary);
            TestTrue(TEXT("The member reference migrates its owner to ROS"), Reference.GetMemberParentClass() == NewLibrary);
            Functions.Add(FunctionName, Function);
        }
    }
    if (Functions.Num() != UE_ARRAY_COUNT(FunctionNames))
    {
        return false;
    }

    // Invoke the resolved UFunctions through reflection, as a migrated Blueprint does. FStructOnScope
    // owns parameter initialization/destruction, including the FString input and return values.
    UObject* LibraryObject = NewLibrary->GetDefaultObject();
    {
        UFunction* Function = Functions.FindChecked(TEXT("CameraLinkToOpticalRotation"));
        FStructOnScope Arguments(Function);
        LibraryObject->ProcessEvent(Function, Arguments.GetStructMemory());
        const FStructProperty* Result = FindFProperty<FStructProperty>(Function, TEXT("ReturnValue"));
        if (TestNotNull(TEXT("Optical rotation return pin survives"), Result))
        {
            const FQuat Rotation = *Result->ContainerPtrToValuePtr<FQuat>(Arguments.GetStructMemory());
            TestTrue(TEXT("Migrated optical rotation maps optical forward to link forward"),
                Rotation.RotateVector(FVector::ZAxisVector).Equals(FVector::XAxisVector, 1.e-9));
            TestTrue(TEXT("Migrated optical rotation maps optical right to link right"),
                Rotation.RotateVector(FVector::XAxisVector).Equals(-FVector::YAxisVector, 1.e-9));
        }
    }
    {
        UFunction* Function = Functions.FindChecked(TEXT("GetOpticalFrameSuffix"));
        FStructOnScope Arguments(Function);
        LibraryObject->ProcessEvent(Function, Arguments.GetStructMemory());
        const FStrProperty* Result = FindFProperty<FStrProperty>(Function, TEXT("ReturnValue"));
        if (TestNotNull(TEXT("Optical suffix return pin survives"), Result))
        {
            TestEqual(TEXT("Migrated optical suffix"), Result->GetPropertyValue_InContainer(Arguments.GetStructMemory()), FString(TEXT("_optical")));
        }
    }
    {
        UFunction* Function = Functions.FindChecked(TEXT("SecondsToRosTime"));
        FStructOnScope Arguments(Function);
        const FDoubleProperty* Seconds = FindFProperty<FDoubleProperty>(Function, TEXT("Seconds"));
        const FIntProperty* Sec = FindFProperty<FIntProperty>(Function, TEXT("Sec"));
        const FIntProperty* Nanosec = FindFProperty<FIntProperty>(Function, TEXT("Nanosec"));
        if (TestNotNull(TEXT("Seconds input pin survives"), Seconds)
            && TestNotNull(TEXT("Sec output pin survives"), Sec) && TestNotNull(TEXT("Nanosec output pin survives"), Nanosec))
        {
            Seconds->SetPropertyValue_InContainer(Arguments.GetStructMemory(), 0.9999999999);
            LibraryObject->ProcessEvent(Function, Arguments.GetStructMemory());
            TestEqual(TEXT("Migrated time conversion carries seconds"), Sec->GetPropertyValue_InContainer(Arguments.GetStructMemory()), 1);
            TestEqual(TEXT("Migrated time conversion carries nanoseconds"), Nanosec->GetPropertyValue_InContainer(Arguments.GetStructMemory()), 0);
        }
    }
    {
        UFunction* Function = Functions.FindChecked(TEXT("RosTimeToSeconds"));
        FStructOnScope Arguments(Function);
        const FIntProperty* Sec = FindFProperty<FIntProperty>(Function, TEXT("Sec"));
        const FIntProperty* Nanosec = FindFProperty<FIntProperty>(Function, TEXT("Nanosec"));
        const FDoubleProperty* Result = FindFProperty<FDoubleProperty>(Function, TEXT("ReturnValue"));
        if (TestNotNull(TEXT("Sec input pin survives"), Sec) && TestNotNull(TEXT("Nanosec input pin survives"), Nanosec)
            && TestNotNull(TEXT("Seconds return pin survives"), Result))
        {
            Sec->SetPropertyValue_InContainer(Arguments.GetStructMemory(), 7);
            Nanosec->SetPropertyValue_InContainer(Arguments.GetStructMemory(), 125000000);
            LibraryObject->ProcessEvent(Function, Arguments.GetStructMemory());
            TestEqual(TEXT("Migrated ROS timestamp"), Result->GetPropertyValue_InContainer(Arguments.GetStructMemory()), 7.125, 0.0);
        }
    }
    {
        UFunction* Function = Functions.FindChecked(TEXT("SanitizeRosName"));
        FStructOnScope Arguments(Function);
        const FStrProperty* Name = FindFProperty<FStrProperty>(Function, TEXT("Name"));
        const FStrProperty* Result = FindFProperty<FStrProperty>(Function, TEXT("ReturnValue"));
        if (TestNotNull(TEXT("Name input pin survives"), Name) && TestNotNull(TEXT("Name return pin survives"), Result))
        {
            Name->SetPropertyValue_InContainer(Arguments.GetStructMemory(), TEXT("Robot A"));
            LibraryObject->ProcessEvent(Function, Arguments.GetStructMemory());
            TestEqual(TEXT("Migrated ROS name"), Result->GetPropertyValue_InContainer(Arguments.GetStructMemory()), FString(TEXT("Robot_A")));
        }
    }
    return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
