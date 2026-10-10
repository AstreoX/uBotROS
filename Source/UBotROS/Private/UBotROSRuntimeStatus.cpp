#include "UBotROSRuntimeStatus.h"

#include "Internationalization/PolyglotTextData.h"
#include "Internationalization/TextLocalizationManager.h"
#include "UBotRuntimeStatus.h"

#define LOCTEXT_NAMESPACE "UBotROS"

// Key, English source text, Simplified Chinese. One list feeds both the LOCTEXTs and their translations.
#define UBOT_ROS_STATUS_TEXTS(Entry) \
    Entry(BridgeLabel, "ROS bridge", "ROS 桥接") \
    Entry(BridgeConnecting, "Connecting", "正在连接") \
    Entry(BridgeConnected, "Connected", "已连接") \
    Entry(BridgeDisconnected, "Disconnected", "已断开") \
    Entry(BridgeUnreachable, "Could not connect to ubot_ros_bridge at {address}.", "无法连接 {address} 上的 ubot_ros_bridge。")

namespace UBot::ROSRuntimeStatus::Private
{
#define UBOT_DEFINE_TEXT(Key, English, Chinese) FText Key() { return LOCTEXT(#Key, English); }
    UBOT_ROS_STATUS_TEXTS(UBOT_DEFINE_TEXT)
#undef UBOT_DEFINE_TEXT

    constexpr const TCHAR* PackageName = TEXT("UBotROS");
    constexpr const TCHAR* UnreachableCode = TEXT("rosBridgeUnreachable");

    /** What one bridge subsystem last published. */
    struct FOwnerState
    {
        const void* Owner = nullptr;
        EUBotROSBridgeStatus Status = EUBotROSBridgeStatus::Disabled;
        FString Endpoint;
        /** Orders the updates of all owners, so that equally ranked bridges are decided by recency. */
        uint64 Sequence = 0;
    };

    // Game thread only, like FUBotRuntimeStatus.
    TArray<FOwnerState>& GetOwnerStates()
    {
        static TArray<FOwnerState> States;
        return States;
    }

    uint64 NextSequence()
    {
        static uint64 Sequence = 0;
        return ++Sequence;
    }

    /** Connected beats connecting beats failed beats disconnected. */
    int32 RankOf(EUBotROSBridgeStatus Status)
    {
        switch (Status)
        {
        case EUBotROSBridgeStatus::Connected:
            return 3;
        case EUBotROSBridgeStatus::Connecting:
            return 2;
        case EUBotROSBridgeStatus::Error:
            return 1;
        case EUBotROSBridgeStatus::Disabled:
        default:
            return 0;
        }
    }

    /** Shows the state that the owners agree on (see the header), or removes everything when there is none. */
    void Republish()
    {
        FUBotRuntimeStatus& RuntimeStatus = FUBotRuntimeStatus::Get();
        const TArray<FOwnerState>& States = GetOwnerStates();
        if (States.IsEmpty())
        {
            RuntimeStatus.RemoveItem(BridgeId);
            RuntimeStatus.ClearProblem(BridgeId);
            return;
        }

        const FOwnerState* Winner = &States[0];
        for (const FOwnerState& State : States)
        {
            const int32 Rank = RankOf(State.Status);
            const int32 WinnerRank = RankOf(Winner->Status);
            if (Rank > WinnerRank || (Rank == WinnerRank && State.Sequence > Winner->Sequence))
            {
                Winner = &State;
            }
        }

        const EUBotROSBridgeStatus Status = Winner->Status;
        const FString& Endpoint = Winner->Endpoint;

        FUBotRuntimeItem Item;
        Item.Id = BridgeId;
        Item.Label = BridgeLabel();
        Item.Detail = Endpoint;
        switch (Status)
        {
        case EUBotROSBridgeStatus::Connecting:
            Item.Value = TEXT("connecting");
            Item.ValueText = BridgeConnecting();
            Item.Severity = EUBotRuntimeSeverity::Info;
            break;
        case EUBotROSBridgeStatus::Connected:
            Item.Value = TEXT("connected");
            Item.ValueText = BridgeConnected();
            Item.Severity = EUBotRuntimeSeverity::Ok;
            break;
        case EUBotROSBridgeStatus::Error:
            Item.Value = TEXT("disconnected");
            Item.ValueText = BridgeDisconnected();
            Item.Severity = EUBotRuntimeSeverity::Warning;
            break;
        case EUBotROSBridgeStatus::Disabled:
        default:
            Item.Value = TEXT("disconnected");
            Item.ValueText = BridgeDisconnected();
            Item.Severity = EUBotRuntimeSeverity::Info;
            break;
        }
        RuntimeStatus.SetItem(Item);

        if (Status == EUBotROSBridgeStatus::Error)
        {
            FUBotRuntimeProblem Problem;
            Problem.Id = BridgeId;
            Problem.Package = PackageName;
            Problem.Severity = EUBotRuntimeSeverity::Info;
            Problem.Code = UnreachableCode;
            Problem.Params.Emplace(TEXT("address"), Endpoint);
            Problem.Message = FText::FormatNamed(BridgeUnreachable(), TEXT("address"), FText::FromString(Endpoint));
            RuntimeStatus.ReportProblem(Problem);
        }
        else if (Status != EUBotROSBridgeStatus::Connecting)
        {
            RuntimeStatus.ClearProblem(BridgeId);
        }
    }
}

void UBot::ROSRuntimeStatus::PublishBridgeStatus(const void* Owner, EUBotROSBridgeStatus Status, const FString& Endpoint)
{
    using namespace UBot::ROSRuntimeStatus::Private;

    TArray<FOwnerState>& States = GetOwnerStates();
    FOwnerState* State = States.FindByPredicate([Owner](const FOwnerState& Other) { return Other.Owner == Owner; });
    if (State == nullptr)
    {
        State = &States.AddDefaulted_GetRef();
        State->Owner = Owner;
    }
    State->Status = Status;
    State->Endpoint = Endpoint;
    State->Sequence = NextSequence();

    Republish();
}

void UBot::ROSRuntimeStatus::ClearBridgeStatus(const void* Owner)
{
    using namespace UBot::ROSRuntimeStatus::Private;

    // A bridge that never published (it was never connected, or another world's bridge was) has
    // nothing to withdraw, and must not disturb what the others show.
    if (GetOwnerStates().RemoveAll([Owner](const FOwnerState& State) { return State.Owner == Owner; }) > 0)
    {
        Republish();
    }
}

int32 UBot::ROSRuntimeStatus::GetPublishingOwnerCount()
{
    return Private::GetOwnerStates().Num();
}

void UBot::ROSRuntimeStatus::RegisterLocalizedText()
{
    TArray<FPolyglotTextData> Translations;
#define UBOT_ADD_TRANSLATION(Key, English, Chinese) \
    Translations.Emplace(ELocalizedTextSourceCategory::Engine, TEXT(LOCTEXT_NAMESPACE), TEXT(#Key), TEXT(English), TEXT("en")); \
    Translations.Last().AddLocalizedString(TEXT("zh-Hans"), TEXT(Chinese));
    UBOT_ROS_STATUS_TEXTS(UBOT_ADD_TRANSLATION)
#undef UBOT_ADD_TRANSLATION

    FTextLocalizationManager::Get().RegisterPolyglotTextData(Translations);
}

#undef UBOT_ROS_STATUS_TEXTS
#undef LOCTEXT_NAMESPACE
