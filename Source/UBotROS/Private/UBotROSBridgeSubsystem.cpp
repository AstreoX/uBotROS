#include "UBotROSBridgeSubsystem.h"

#include "AddressInfoTypes.h"
#include "Containers/Deque.h"
#include "Containers/Queue.h"
#include "Engine/Engine.h"
#include "Engine/World.h"
#include "HAL/Event.h"
#include "HAL/PlatformProcess.h"
#include "HAL/PlatformTime.h"
#include "HAL/Runnable.h"
#include "HAL/RunnableThread.h"
#include "IPAddress.h"
#include "Interfaces/IPv4/IPv4Address.h"
#include "Misc/ScopeLock.h"
#include "SocketSubsystem.h"
#include "Sockets.h"
#include "SocketTypes.h"
#include "UBotClock.h"
#include "UBotROS.h"
#include "UBotROSProtocol.h"
#include "UBotROSRuntimeStatus.h"
#include "UBotROSSettings.h"
#include "UBotRosConventions.h"

#include <atomic>

namespace UBot::ROSBridgeSubsystem::Private
{
    /** Upper bound for one connection attempt (name resolution excluded). */
    constexpr double ConnectTimeoutSeconds = 3.0;
    /** Granularity of the I/O thread's waits; bounds the latency of newly queued frames and of stop requests. */
    constexpr int32 SocketWaitMilliseconds = 2;
    constexpr int32 ConnectWaitMilliseconds = 20;
    /** Granularity at which the I/O thread polls a pending name lookup (and checks for a stop request). */
    constexpr int32 ResolveWaitMilliseconds = 20;
    /** A lookup that has not finished after this long is reported once as a failed attempt (it keeps running). */
    constexpr double ResolveReportSeconds = 3.0;
    constexpr int32 ReceiveBufferBytes = 256 * 1024;
    constexpr int32 SocketBufferBytes = 1024 * 1024;
    /** Frames up to this size are coalesced into one staging buffer (fewer send calls and packets). */
    constexpr int64 SmallFrameBytes = 16 * 1024;
    /** Coalescing stops once the staging buffer holds this much. */
    constexpr int32 StagingTargetBytes = 64 * 1024;
    /** Largest single send call. */
    constexpr int32 MaxSendChunkBytes = 1024 * 1024;
    /** Bytes written per I/O loop iteration before the thread polls for received data again. */
    constexpr int64 SendBudgetBytes = 4 * 1024 * 1024;
    /** Receive calls per I/O loop iteration before the thread sends again. */
    constexpr int32 ReceiveCallsPerIteration = 16;
    /** Events processed per Tick; the rest wait for the next Tick. */
    constexpr int32 MaxEventsPerTick = 8192;
    constexpr double RejectWarningIntervalSeconds = 1.0;
    constexpr int64 MinQueuedBytes = 1024;
    constexpr int32 MinPayloadBytes = 1024;

    const TCHAR* const TfTopic = TEXT("/tf");
    const TCHAR* const TfStaticTopic = TEXT("/tf_static");

    bool IsReliable(const FUBotROSQos& Qos)
    {
        return Qos.Reliability == EUBotROSReliability::Reliable;
    }

    bool SameTransform(const FUBotRosTransformStamped& A, const FUBotRosTransformStamped& B)
    {
        const FUBotRosTransform& TA = A.transform;
        const FUBotRosTransform& TB = B.transform;
        return A.header.frame_id.Equals(B.header.frame_id, ESearchCase::CaseSensitive)
            && TA.translation.x == TB.translation.x && TA.translation.y == TB.translation.y && TA.translation.z == TB.translation.z
            && TA.rotation.x == TB.rotation.x && TA.rotation.y == TB.rotation.y && TA.rotation.z == TB.rotation.z && TA.rotation.w == TB.rotation.w;
    }
}

/** What the I/O thread reports to the game thread. */
enum class EUBotROSBridgeEventType : uint8
{
    /** A connection attempt failed (Text = reason). */
    AttemptFailed,
    /** HELLO_ACK accepted (Text = server, Detail = ROS distro). */
    HandshakeComplete,
    /** A TCP session ended (Text = reason). */
    SessionEnded,
    /** MESSAGE (Id = subscription id, Payload = complete MESSAGE payload incl. the 4-byte id). */
    Message,
    /** SUB_COUNT (Counts). */
    SubscriberCounts,
    /** ERROR frame from the bridge (Id if bHasId, Detail = code, Text = message). */
    BridgeError
};

struct FUBotROSBridgeEvent
{
    EUBotROSBridgeEventType Type = EUBotROSBridgeEventType::AttemptFailed;
    uint32 Id = 0;
    bool bHasId = false;
    FString Text;
    FString Detail;
    TArray<uint8> Payload;
    TArray<TPair<uint32, int32>> Counts;
};

/** One frame waiting to be written: Head (header, id prefix and - for small frames - the body), then Body. */
struct FUBotROSOutgoingFrame
{
    UBot::ROSProtocol::EOp Op = UBot::ROSProtocol::EOp::Ping;
    /** Publisher id of PUBLISH frames. */
    uint32 Id = 0;
    TArray<uint8> Head;
    TArray<uint8> Body;

    int64 Num() const
    {
        return static_cast<int64>(Head.Num()) + Body.Num();
    }
};

enum class EUBotROSEnqueueResult : uint8
{
    Queued,
    Dropped,
    RejectedOversize,
    RejectedQueueFull,
    UnknownId
};

/** Result slot of one asynchronous name lookup; shared with the pool thread that fills it (it may outlive the client). */
struct FUBotROSResolveState
{
    explicit FUBotROSResolveState(const FString& InHost)
        : Result(*InHost, TEXT(""))
        , StartedSeconds(FPlatformTime::Seconds())
    {
    }

    /** Written by the lookup thread before bDone is set, read by the I/O thread after it. */
    FAddressInfoResult Result;
    std::atomic<bool> bDone { false };
    double StartedSeconds = 0.0;
    /** I/O thread only. */
    bool bReportedSlow = false;
};

enum class EUBotROSResolveStatus : uint8
{
    Ready,
    Pending,
    Failed
};

struct FUBotROSClientConfig
{
    FString Host;
    int32 Port = UBot::ROSProtocol::DefaultPort;
    double ReconnectIntervalSeconds = 2.0;
    uint32 MaxPayloadBytes = UBot::ROSProtocol::DefaultMaxPayloadBytes;
    int64 MaxQueuedBytes = 268435456;
    FString ClientName;
};

/**
 * Socket side of the bridge subsystem. The registry (advertisements, subscriptions) and the outgoing
 * queues are shared with the game thread under Mutex; the socket, the parser and the send buffers
 * belong to the I/O thread. Results travel to the game thread through the Events queue.
 */
class FUBotROSBridgeClient final : public FRunnable
{
public:
    UE_NONCOPYABLE(FUBotROSBridgeClient);

    FUBotROSBridgeClient()
        : WakeEvent(FPlatformProcess::GetSynchEventFromPool(false))
    {
    }

    virtual ~FUBotROSBridgeClient() override
    {
        Shutdown();
        ClearEvents();
        FPlatformProcess::ReturnSynchEventToPool(WakeEvent);
        WakeEvent = nullptr;
    }

    // ---------------------------------------------------------------------------------------------
    // Game thread
    // ---------------------------------------------------------------------------------------------

    bool Start(const FUBotROSClientConfig& InConfig, FString& OutError)
    {
        check(Thread == nullptr);

        SocketSubsystem = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM);
        if (SocketSubsystem == nullptr)
        {
            OutError = TEXT("no platform socket subsystem is available");
            return false;
        }

        Config = InConfig;
        {
            FScopeLock Lock(&Mutex);
            MaxPayloadBytes = Config.MaxPayloadBytes;
            MaxQueuedBytes = Config.MaxQueuedBytes;
            bWanted = true;
            bSessionActive = false;
        }
        ReceiveBuffer.SetNumUninitialized(UBot::ROSBridgeSubsystem::Private::ReceiveBufferBytes);
        bStopRequested = false;
        // A previous run may have left a lookup or an address for another host behind.
        PendingResolve.Reset();
        CachedAddress.Reset();

        Thread = FRunnableThread::Create(this, TEXT("UBotROSBridgeIO"), 0, TPri_Normal);
        if (Thread == nullptr)
        {
            FScopeLock Lock(&Mutex);
            bWanted = false;
            OutError = TEXT("could not create the bridge I/O thread (multithreading unavailable?)");
            return false;
        }
        return true;
    }

    /**
     * Stops and joins the I/O thread (bounded by ConnectWaitMilliseconds / SocketWaitMilliseconds /
     * ResolveWaitMilliseconds; a running name lookup lives on a pool thread and is simply abandoned)
     * and discards all queued frames.
     */
    void Shutdown()
    {
        if (Thread != nullptr)
        {
            Thread->Kill(true);
            delete Thread;
            Thread = nullptr;
        }

        FScopeLock Lock(&Mutex);
        bWanted = false;
        bSessionActive = false;
        DroppedMessages += CountQueuedPublishesLocked();
        OrderedQueue.Empty();
        BestEffortSlots.Empty();
        BestEffortOrder.Empty();
        QueuedBytes = 0;
    }

    bool IsRunning() const
    {
        return Thread != nullptr;
    }

    void AddPublisher(uint32 Id, const FString& Topic, const FString& Type, const FUBotROSQos& Qos)
    {
        using namespace UBot::ROSProtocol;
        FScopeLock Lock(&Mutex);
        PublisherRegistry.Add(Id, FRegisteredTopic{ Topic, Type, Qos });
        if (bSessionActive)
        {
            EnqueueControlLocked(EOp::Advertise, MakeTopicJson(Id, Topic, Type, Qos));
        }
    }

    void RemovePublisher(uint32 Id)
    {
        using namespace UBot::ROSProtocol;
        FScopeLock Lock(&Mutex);
        if (PublisherRegistry.Remove(Id) == 0)
        {
            return;
        }
        RemoveBestEffortSlotLocked(Id, true);
        if (bSessionActive)
        {
            // Reliable messages already queued for Id precede the UNADVERTISE and are still delivered.
            EnqueueControlLocked(EOp::Unadvertise, MakeIdJson(Id));
        }
        else
        {
            // Not advertised again after a reconnect, so its queued messages could not be delivered.
            PurgeOrderedQueueLocked();
        }
    }

    void AddSubscription(uint32 Id, const FString& Topic, const FString& Type, const FUBotROSQos& Qos)
    {
        using namespace UBot::ROSProtocol;
        FScopeLock Lock(&Mutex);
        SubscriptionRegistry.Add(Id, FRegisteredTopic{ Topic, Type, Qos });
        if (bSessionActive)
        {
            EnqueueControlLocked(EOp::Subscribe, MakeTopicJson(Id, Topic, Type, Qos));
        }
    }

    void RemoveSubscription(uint32 Id)
    {
        using namespace UBot::ROSProtocol;
        FScopeLock Lock(&Mutex);
        if (SubscriptionRegistry.Remove(Id) > 0 && bSessionActive)
        {
            EnqueueControlLocked(EOp::Unsubscribe, MakeIdJson(Id));
        }
    }

    EUBotROSEnqueueResult EnqueuePublish(uint32 Id, TArray<uint8>&& Cdr)
    {
        FScopeLock Lock(&Mutex);
        const FRegisteredTopic* Publisher = PublisherRegistry.Find(Id);
        if (Publisher == nullptr)
        {
            return EUBotROSEnqueueResult::UnknownId;
        }
        if (static_cast<int64>(Cdr.Num()) + UBot::ROSProtocol::IdPrefixSize > static_cast<int64>(MaxPayloadBytes))
        {
            ++RejectedMessages;
            return EUBotROSEnqueueResult::RejectedOversize;
        }
        if (!bWanted)
        {
            ++DroppedMessages;
            return EUBotROSEnqueueResult::Dropped;
        }

        const bool bReliable = UBot::ROSBridgeSubsystem::Private::IsReliable(Publisher->Qos);
        if (!bReliable && !bSessionActive)
        {
            ++DroppedMessages;
            return EUBotROSEnqueueResult::Dropped;
        }

        FUBotROSOutgoingFrame Frame = MakePublishFrame(Id, MoveTemp(Cdr));
        const int64 Size = Frame.Num();

        if (bReliable)
        {
            if (QueuedBytes + Size > MaxQueuedBytes)
            {
                EvictBestEffortLocked(QueuedBytes + Size - MaxQueuedBytes, 0, false);
            }
            if (QueuedBytes + Size > MaxQueuedBytes)
            {
                ++RejectedMessages;
                return EUBotROSEnqueueResult::RejectedQueueFull;
            }
            OrderedQueue.EmplaceLast(MoveTemp(Frame));
            QueuedBytes += Size;
            return EUBotROSEnqueueResult::Queued;
        }

        // best_effort: keep only the newest pending message of this publisher.
        FUBotROSOutgoingFrame* Existing = BestEffortSlots.Find(Id);
        const int64 ExistingSize = Existing != nullptr ? Existing->Num() : 0;
        if (QueuedBytes - ExistingSize + Size > MaxQueuedBytes)
        {
            EvictBestEffortLocked(QueuedBytes - ExistingSize + Size - MaxQueuedBytes, Id, true);
        }
        if (QueuedBytes - ExistingSize + Size > MaxQueuedBytes)
        {
            ++DroppedMessages;
            return EUBotROSEnqueueResult::Dropped;
        }

        Existing = BestEffortSlots.Find(Id);
        if (Existing != nullptr)
        {
            ++DroppedMessages;
            QueuedBytes -= Existing->Num();
            *Existing = MoveTemp(Frame);
        }
        else
        {
            BestEffortSlots.Add(Id, MoveTemp(Frame));
            BestEffortOrder.Add(Id);
        }
        QueuedBytes += Size;
        return EUBotROSEnqueueResult::Queued;
    }

    bool PopEvent(FUBotROSBridgeEvent& OutEvent)
    {
        if (!Events.Dequeue(OutEvent))
        {
            return false;
        }
        if (OutEvent.Type == EUBotROSBridgeEventType::Message)
        {
            --PendingIncomingMessages;
            PendingIncomingBytes -= OutEvent.Payload.Num();
        }
        return true;
    }

    /** Discards undelivered events. Call only while the I/O thread is stopped. */
    void ClearEvents()
    {
        FUBotROSBridgeEvent Event;
        while (PopEvent(Event))
        {
        }
    }

    /** Once this returns with bPaused, no frame queued afterwards is taken for sending until unpaused. */
    void SetSendingPaused(bool bPaused)
    {
        FScopeLock Lock(&Mutex);
        bSendingPaused = bPaused;
    }

    void FillStats(FUBotROSBridgeStats& Out) const
    {
        Out.FramesSent = FramesSent.load();
        Out.FramesReceived = FramesReceived.load();
        Out.BytesSent = BytesSent.load();
        Out.BytesReceived = BytesReceived.load();
        Out.DroppedIncomingMessages = DroppedIncomingMessages.load();
        Out.PendingIncomingMessages = PendingIncomingMessages.load();
        Out.SessionCount = SessionCount.load();

        FScopeLock Lock(&Mutex);
        Out.DroppedMessages = DroppedMessages;
        Out.RejectedMessages = RejectedMessages;
        Out.QueuedBytes = QueuedBytes;
    }

    // ---------------------------------------------------------------------------------------------
    // FRunnable (I/O thread)
    // ---------------------------------------------------------------------------------------------

    virtual uint32 Run() override
    {
        double NextAttemptSeconds = 0.0;
        while (!bStopRequested)
        {
            const double Now = FPlatformTime::Seconds();
            if (Now < NextAttemptSeconds)
            {
                WakeEvent->Wait(FTimespan::FromSeconds(FMath::Min(NextAttemptSeconds - Now, 0.25)));
                continue;
            }

            FString Failure;
            TSharedPtr<FInternetAddr> Address;
            const EUBotROSResolveStatus Resolved = ResolveAddress(Address, Failure);
            if (Resolved == EUBotROSResolveStatus::Pending)
            {
                // The lookup runs on a pool thread; keep the stop request responsive meanwhile.
                WakeEvent->Wait(FTimespan::FromMilliseconds(UBot::ROSBridgeSubsystem::Private::ResolveWaitMilliseconds));
                continue;
            }

            Socket = Resolved == EUBotROSResolveStatus::Ready ? OpenSocket(*Address, Failure) : nullptr;
            if (Socket == nullptr)
            {
                // The next attempt looks the host up again (its address may have changed).
                CachedAddress.Reset();
                if (!bStopRequested)
                {
                    PostEvent(EUBotROSBridgeEventType::AttemptFailed, Failure);
                }
                NextAttemptSeconds = FPlatformTime::Seconds() + Config.ReconnectIntervalSeconds;
                continue;
            }

            BeginSession();
            const FString Reason = RunSession();
            EndSession(Reason);
            NextAttemptSeconds = FPlatformTime::Seconds() + Config.ReconnectIntervalSeconds;
        }
        return 0;
    }

    virtual void Stop() override
    {
        bStopRequested = true;
        WakeEvent->Trigger();
    }

private:
    struct FRegisteredTopic
    {
        FString Topic;
        FString Type;
        FUBotROSQos Qos;
    };

    // ---------------------------------------------------------------------------------------------
    // Queues (Mutex held)
    // ---------------------------------------------------------------------------------------------

    static FUBotROSOutgoingFrame MakeJsonFrame(UBot::ROSProtocol::EOp Op, const FString& Json)
    {
        FUBotROSOutgoingFrame Frame;
        Frame.Op = Op;
        Frame.Head = UBot::ROSProtocol::BuildJsonFrame(Op, Json);
        return Frame;
    }

    static FUBotROSOutgoingFrame MakeEmptyFrame(UBot::ROSProtocol::EOp Op)
    {
        FUBotROSOutgoingFrame Frame;
        Frame.Op = Op;
        Frame.Head.SetNumUninitialized(UBot::ROSProtocol::HeaderSize);
        UBot::ROSProtocol::WriteHeader(Frame.Head.GetData(), Op, 0);
        return Frame;
    }

    static FUBotROSOutgoingFrame MakePublishFrame(uint32 Id, TArray<uint8>&& Cdr)
    {
        using namespace UBot::ROSProtocol;
        FUBotROSOutgoingFrame Frame;
        Frame.Op = EOp::Publish;
        Frame.Id = Id;
        const int32 PrefixSize = HeaderSize + IdPrefixSize;
        if (Cdr.Num() <= UBot::ROSBridgeSubsystem::Private::SmallFrameBytes)
        {
            Frame.Head.SetNumUninitialized(PrefixSize + Cdr.Num());
            if (Cdr.Num() > 0)
            {
                FMemory::Memcpy(Frame.Head.GetData() + PrefixSize, Cdr.GetData(), Cdr.Num());
            }
        }
        else
        {
            Frame.Head.SetNumUninitialized(PrefixSize);
            Frame.Body = MoveTemp(Cdr);
        }
        const int32 BodySize = Frame.Body.Num() > 0 ? Frame.Body.Num() : Frame.Head.Num() - PrefixSize;
        WriteHeader(Frame.Head.GetData(), EOp::Publish, static_cast<uint32>(IdPrefixSize + BodySize));
        WriteU32LE(Frame.Head.GetData() + HeaderSize, Id);
        return Frame;
    }

    void EnqueueControlLocked(UBot::ROSProtocol::EOp Op, const FString& Json)
    {
        FUBotROSOutgoingFrame Frame = MakeJsonFrame(Op, Json);
        QueuedBytes += Frame.Num();
        OrderedQueue.EmplaceLast(MoveTemp(Frame));
    }

    /**
     * PING / PONG. After the handshake they overtake queued data (they carry no id, so order does not
     * matter), which keeps a busy connection from looking idle to the peer.
     */
    void EnqueueKeepalive(UBot::ROSProtocol::EOp Op)
    {
        FScopeLock Lock(&Mutex);
        FUBotROSOutgoingFrame Frame = MakeEmptyFrame(Op);
        QueuedBytes += Frame.Num();
        if (bHandshakeDone)
        {
            OrderedQueue.EmplaceFirst(MoveTemp(Frame));
        }
        else
        {
            OrderedQueue.EmplaceLast(MoveTemp(Frame));
        }
    }

    void RemoveBestEffortSlotLocked(uint32 Id, bool bCountAsDropped)
    {
        FUBotROSOutgoingFrame Removed;
        if (BestEffortSlots.RemoveAndCopyValue(Id, Removed))
        {
            QueuedBytes -= Removed.Num();
            BestEffortOrder.Remove(Id);
            if (bCountAsDropped)
            {
                ++DroppedMessages;
            }
        }
    }

    /** Drops the oldest pending best_effort messages (except ExceptId) until BytesNeeded bytes are freed. */
    void EvictBestEffortLocked(int64 BytesNeeded, uint32 ExceptId, bool bHasException)
    {
        int64 Freed = 0;
        for (int32 Index = 0; Index < BestEffortOrder.Num() && Freed < BytesNeeded;)
        {
            const uint32 Id = BestEffortOrder[Index];
            if (bHasException && Id == ExceptId)
            {
                ++Index;
                continue;
            }
            FUBotROSOutgoingFrame Removed;
            if (BestEffortSlots.RemoveAndCopyValue(Id, Removed))
            {
                Freed += Removed.Num();
                QueuedBytes -= Removed.Num();
                ++DroppedMessages;
            }
            BestEffortOrder.RemoveAt(Index);
        }
    }

    /**
     * Keeps only the PUBLISH frames of registered publishers (in order); control frames are dropped
     * because the registry is re-sent after the next HELLO.
     */
    void PurgeOrderedQueueLocked()
    {
        TDeque<FUBotROSOutgoingFrame> Kept;
        FUBotROSOutgoingFrame Frame;
        while (OrderedQueue.TryPopFirst(Frame))
        {
            if (Frame.Op == UBot::ROSProtocol::EOp::Publish)
            {
                if (PublisherRegistry.Contains(Frame.Id))
                {
                    Kept.EmplaceLast(MoveTemp(Frame));
                    continue;
                }
                ++DroppedMessages;
            }
            QueuedBytes -= Frame.Num();
        }
        OrderedQueue = MoveTemp(Kept);
    }

    int64 CountQueuedPublishesLocked() const
    {
        int64 Count = BestEffortSlots.Num();
        for (const FUBotROSOutgoingFrame& Frame : OrderedQueue)
        {
            Count += Frame.Op == UBot::ROSProtocol::EOp::Publish ? 1 : 0;
        }
        return Count;
    }

    /** Ordered queue (control + reliable) first, then pending best_effort messages oldest first. */
    bool PopNextFrameLocked(FUBotROSOutgoingFrame& OutFrame)
    {
        if (OrderedQueue.TryPopFirst(OutFrame))
        {
            QueuedBytes -= OutFrame.Num();
            return true;
        }
        while (BestEffortOrder.Num() > 0)
        {
            const uint32 Id = BestEffortOrder[0];
            BestEffortOrder.RemoveAt(0);
            if (BestEffortSlots.RemoveAndCopyValue(Id, OutFrame))
            {
                QueuedBytes -= OutFrame.Num();
                return true;
            }
        }
        return false;
    }

    TArray<uint32> SortedKeys(const TMap<uint32, FRegisteredTopic>& Registry) const
    {
        TArray<uint32> Keys;
        Registry.GetKeys(Keys);
        Keys.Sort();
        return Keys;
    }

    // ---------------------------------------------------------------------------------------------
    // I/O thread
    // ---------------------------------------------------------------------------------------------

    void PostEvent(EUBotROSBridgeEventType Type, const FString& Text, const FString& Detail = FString())
    {
        FUBotROSBridgeEvent Event;
        Event.Type = Type;
        Event.Text = Text;
        Event.Detail = Detail;
        Events.Enqueue(MoveTemp(Event));
    }

    /**
     * IP literals are parsed in place. A host name is looked up on a thread-pool thread (a plain
     * getaddrinfo cannot be interrupted, and joining this thread must never wait for it): the first
     * call starts the lookup and returns Pending, later calls poll it. A resolved address is kept
     * for the following attempts until one of them fails.
     */
    EUBotROSResolveStatus ResolveAddress(TSharedPtr<FInternetAddr>& OutAddress, FString& OutError)
    {
        using namespace UBot::ROSBridgeSubsystem::Private;

        if (CachedAddress.IsValid())
        {
            OutAddress = CachedAddress;
            return EUBotROSResolveStatus::Ready;
        }

        // Only IP literals are parsed in place (GetAddressFromString logs a warning for every host name).
        TSharedPtr<FInternetAddr> Address;
        FIPv4Address Ipv4Literal;
        if (Config.Host.Contains(TEXT(":")) || FIPv4Address::Parse(Config.Host, Ipv4Literal))
        {
            Address = SocketSubsystem->GetAddressFromString(Config.Host);
            if (Address.IsValid())
            {
                Address->SetPort(Config.Port);
                OutAddress = Address;
                return EUBotROSResolveStatus::Ready;
            }
            OutError = FString::Printf(TEXT("'%s' is not a valid IP address"), *Config.Host);
            return EUBotROSResolveStatus::Failed;
        }

        if (!PendingResolve.IsValid())
        {
            PendingResolve = MakeShared<FUBotROSResolveState, ESPMode::ThreadSafe>(Config.Host);
            const TSharedRef<FUBotROSResolveState, ESPMode::ThreadSafe> State = PendingResolve.ToSharedRef();
            // The callback captures only the shared state, never this client: it may run after the client is gone.
            SocketSubsystem->GetAddressInfoAsync([State](FAddressInfoResult Result)
            {
                State->Result = MoveTemp(Result);
                State->bDone.store(true, std::memory_order_release);
            }, *Config.Host, nullptr, EAddressInfoFlags::Default, NAME_None, SOCKTYPE_Streaming);
        }

        FUBotROSResolveState& State = *PendingResolve;
        if (!State.bDone.load(std::memory_order_acquire))
        {
            if (!State.bReportedSlow && FPlatformTime::Seconds() - State.StartedSeconds >= ResolveReportSeconds)
            {
                // Reported once; the lookup keeps running and the next poll still picks up its result.
                State.bReportedSlow = true;
                PostEvent(EUBotROSBridgeEventType::AttemptFailed, FString::Printf(TEXT("the name lookup for host '%s' is taking longer than %.0f s"), *Config.Host, ResolveReportSeconds));
            }
            return EUBotROSResolveStatus::Pending;
        }

        const TSharedPtr<FUBotROSResolveState, ESPMode::ThreadSafe> Done = MoveTemp(PendingResolve);
        PendingResolve.Reset();
        const FAddressInfoResult& Result = Done->Result;
        if (Result.ReturnCode != SE_NO_ERROR || Result.Results.Num() == 0)
        {
            OutError = FString::Printf(TEXT("cannot resolve host '%s' (%s)"), *Config.Host, SocketSubsystem->GetSocketError(Result.ReturnCode));
            return EUBotROSResolveStatus::Failed;
        }
        // The bridge listens on IPv4 by default, so prefer an IPv4 result ("localhost" may list ::1 first).
        int32 Chosen = 0;
        for (int32 Index = 0; Index < Result.Results.Num(); ++Index)
        {
            if (Result.Results[Index].Address->GetProtocolType() == FNetworkProtocolTypes::IPv4)
            {
                Chosen = Index;
                break;
            }
        }
        Address = Result.Results[Chosen].Address->Clone();
        Address->SetPort(Config.Port);
        CachedAddress = Address;
        OutAddress = Address;
        return EUBotROSResolveStatus::Ready;
    }

    FSocket* OpenSocket(const FInternetAddr& Address, FString& OutError)
    {
        using namespace UBot::ROSBridgeSubsystem::Private;

        const FString Endpoint = Address.ToString(true);

        FSocket* NewSocket = SocketSubsystem->CreateSocket(NAME_Stream, TEXT("uBot ROS bridge"), Address.GetProtocolType());
        if (NewSocket == nullptr)
        {
            OutError = FString::Printf(TEXT("cannot create a TCP socket for %s"), *Endpoint);
            return nullptr;
        }

        NewSocket->SetNonBlocking(true);
        NewSocket->SetNoDelay(true);
        int32 ActualSize = 0;
        NewSocket->SetSendBufferSize(SocketBufferBytes, ActualSize);
        NewSocket->SetReceiveBufferSize(SocketBufferBytes, ActualSize);

        if (!NewSocket->Connect(Address))
        {
            OutError = FString::Printf(TEXT("connect to %s failed (%s)"), *Endpoint, SocketSubsystem->GetSocketError(SocketSubsystem->GetLastErrorCode()));
            DestroySocket(NewSocket);
            return nullptr;
        }

        const double Deadline = FPlatformTime::Seconds() + ConnectTimeoutSeconds;
        for (;;)
        {
            if (bStopRequested)
            {
                OutError = TEXT("stopped");
                DestroySocket(NewSocket);
                return nullptr;
            }
            if (NewSocket->Wait(ESocketWaitConditions::WaitForWrite, FTimespan::FromMilliseconds(ConnectWaitMilliseconds)))
            {
                if (NewSocket->GetConnectionState() == SCS_Connected)
                {
                    return NewSocket;
                }
                OutError = FString::Printf(TEXT("connection to %s refused"), *Endpoint);
                DestroySocket(NewSocket);
                return nullptr;
            }
            if (NewSocket->GetConnectionState() == SCS_ConnectionError)
            {
                OutError = FString::Printf(TEXT("connection to %s refused"), *Endpoint);
                DestroySocket(NewSocket);
                return nullptr;
            }
            if (FPlatformTime::Seconds() >= Deadline)
            {
                OutError = FString::Printf(TEXT("connection to %s timed out after %.0f s"), *Endpoint, ConnectTimeoutSeconds);
                DestroySocket(NewSocket);
                return nullptr;
            }
        }
    }

    void DestroySocket(FSocket* InSocket) const
    {
        if (InSocket != nullptr)
        {
            InSocket->Close();
            SocketSubsystem->DestroySocket(InSocket);
        }
    }

    /** HELLO first, then every registered ADVERTISE and SUBSCRIBE, then the reliable messages queued meanwhile. */
    void BeginSession()
    {
        using namespace UBot::ROSProtocol;

        Parser.Reset();
        Parser.SetMaxPayloadBytes(Config.MaxPayloadBytes);
        Staging.Reset();
        StagingOffset = 0;
        StagingFrames = 0;
        StagingPublishFrames = 0;
        LargeFrame = FUBotROSOutgoingFrame();
        bHasLargeFrame = false;
        LargeFrameOffset = 0;
        bHandshakeDone = false;
        const double Now = FPlatformTime::Seconds();
        LastFrameReceivedSeconds = Now;
        LastPingSentSeconds = Now;

        FScopeLock Lock(&Mutex);
        // Leftover control frames are stale: the registry below is the source of truth.
        PurgeOrderedQueueLocked();

        TDeque<FUBotROSOutgoingFrame> Session;
        int64 SessionBytes = 0;
        auto AddControl = [&Session, &SessionBytes](FUBotROSOutgoingFrame&& Frame)
        {
            SessionBytes += Frame.Num();
            Session.EmplaceLast(MoveTemp(Frame));
        };
        AddControl(MakeJsonFrame(EOp::Hello, MakeHelloJson(Config.ClientName)));
        for (const uint32 Id : SortedKeys(PublisherRegistry))
        {
            const FRegisteredTopic& Entry = PublisherRegistry.FindChecked(Id);
            AddControl(MakeJsonFrame(EOp::Advertise, MakeTopicJson(Id, Entry.Topic, Entry.Type, Entry.Qos)));
        }
        for (const uint32 Id : SortedKeys(SubscriptionRegistry))
        {
            const FRegisteredTopic& Entry = SubscriptionRegistry.FindChecked(Id);
            AddControl(MakeJsonFrame(EOp::Subscribe, MakeTopicJson(Id, Entry.Topic, Entry.Type, Entry.Qos)));
        }
        FUBotROSOutgoingFrame Frame;
        while (OrderedQueue.TryPopFirst(Frame))
        {
            Session.EmplaceLast(MoveTemp(Frame));
        }
        OrderedQueue = MoveTemp(Session);
        QueuedBytes += SessionBytes;
        bSessionActive = true;
        ++SessionCount;
    }

    void EndSession(const FString& Reason)
    {
        {
            FScopeLock Lock(&Mutex);
            bSessionActive = false;
            // Frames handed to the socket but not completely written are lost with the connection.
            DroppedMessages += StagingPublishFrames + ((bHasLargeFrame && LargeFrame.Op == UBot::ROSProtocol::EOp::Publish) ? 1 : 0);
            PurgeOrderedQueueLocked();
            DroppedMessages += BestEffortSlots.Num();
            for (const TPair<uint32, FUBotROSOutgoingFrame>& Slot : BestEffortSlots)
            {
                QueuedBytes -= Slot.Value.Num();
            }
            BestEffortSlots.Empty();
            BestEffortOrder.Empty();
        }

        Staging.Empty();
        StagingOffset = 0;
        StagingFrames = 0;
        StagingPublishFrames = 0;
        LargeFrame = FUBotROSOutgoingFrame();
        bHasLargeFrame = false;
        LargeFrameOffset = 0;
        Parser.Reset();

        DestroySocket(Socket);
        Socket = nullptr;

        PostEvent(EUBotROSBridgeEventType::SessionEnded, Reason);
    }

    FString RunSession()
    {
        using namespace UBot::ROSBridgeSubsystem::Private;
        using namespace UBot::ROSProtocol;

        while (!bStopRequested)
        {
            bool bProgress = false;
            FString Error;
            if (!PumpSend(bProgress, Error))
            {
                return Error;
            }
            if (!PumpReceive(bProgress, Error))
            {
                return Error;
            }

            const double Now = FPlatformTime::Seconds();
            if (Now - LastPingSentSeconds >= PingIntervalSeconds)
            {
                EnqueueKeepalive(EOp::Ping);
                LastPingSentSeconds = Now;
            }
            if (Now - LastFrameReceivedSeconds >= IdleTimeoutSeconds)
            {
                return FString::Printf(TEXT("nothing received from the bridge for %.0f s"), IdleTimeoutSeconds);
            }

            if (!bProgress)
            {
                const bool bBlockedOnWrite = StagingOffset < Staging.Num() || bHasLargeFrame;
                Socket->Wait(bBlockedOnWrite ? ESocketWaitConditions::WaitForReadOrWrite : ESocketWaitConditions::WaitForRead,
                    FTimespan::FromMilliseconds(SocketWaitMilliseconds));
            }
        }
        return TEXT("disconnect requested");
    }

    /** Writes a part of Data. OutSent is 0 when the socket would block. False on a socket error. */
    bool SendSome(const uint8* Data, int64 Num, int32& OutSent, FString& OutError)
    {
        OutSent = 0;
        const int32 Chunk = static_cast<int32>(FMath::Min<int64>(Num, UBot::ROSBridgeSubsystem::Private::MaxSendChunkBytes));
        int32 Sent = 0;
        if (!Socket->Send(Data, Chunk, Sent))
        {
            const ESocketErrors Error = SocketSubsystem->GetLastErrorCode();
            if (Error == SE_EWOULDBLOCK || Error == SE_TRY_AGAIN || Error == SE_NO_ERROR)
            {
                return true;
            }
            OutError = FString::Printf(TEXT("send to the bridge failed (%s)"), SocketSubsystem->GetSocketError(Error));
            return false;
        }
        OutSent = FMath::Max(Sent, 0);
        BytesSent += OutSent;
        return true;
    }

    /** Moves frames from the shared queues into the staging buffer (small frames) or LargeFrame. */
    bool RefillSendBuffers()
    {
        using namespace UBot::ROSBridgeSubsystem::Private;

        FScopeLock Lock(&Mutex);
        if (bSendingPaused)
        {
            return false;
        }
        bool bAny = false;
        FUBotROSOutgoingFrame Frame;
        while (PopNextFrameLocked(Frame))
        {
            bAny = true;
            if (Frame.Num() <= SmallFrameBytes)
            {
                Staging.Append(Frame.Head);
                Staging.Append(Frame.Body);
                ++StagingFrames;
                StagingPublishFrames += Frame.Op == UBot::ROSProtocol::EOp::Publish ? 1 : 0;
                if (Staging.Num() >= StagingTargetBytes)
                {
                    break;
                }
            }
            else
            {
                // Sent after the staging buffer, which holds the frames popped before it.
                LargeFrame = MoveTemp(Frame);
                bHasLargeFrame = true;
                LargeFrameOffset = 0;
                break;
            }
        }
        return bAny;
    }

    bool PumpSend(bool& bProgress, FString& OutError)
    {
        using namespace UBot::ROSBridgeSubsystem::Private;

        int64 Budget = SendBudgetBytes;
        while (Budget > 0)
        {
            if (StagingOffset < Staging.Num())
            {
                int32 Sent = 0;
                if (!SendSome(Staging.GetData() + StagingOffset, Staging.Num() - StagingOffset, Sent, OutError))
                {
                    return false;
                }
                if (Sent == 0)
                {
                    return true;
                }
                bProgress = true;
                Budget -= Sent;
                StagingOffset += Sent;
                if (StagingOffset == Staging.Num())
                {
                    FramesSent += StagingFrames;
                    StagingFrames = 0;
                    StagingPublishFrames = 0;
                    StagingOffset = 0;
                    Staging.Reset();
                }
                continue;
            }

            if (bHasLargeFrame)
            {
                const int64 HeadSize = LargeFrame.Head.Num();
                const int64 Total = LargeFrame.Num();
                const uint8* Data = LargeFrameOffset < HeadSize
                    ? LargeFrame.Head.GetData() + LargeFrameOffset
                    : LargeFrame.Body.GetData() + (LargeFrameOffset - HeadSize);
                const int64 Remaining = LargeFrameOffset < HeadSize ? HeadSize - LargeFrameOffset : Total - LargeFrameOffset;
                int32 Sent = 0;
                if (!SendSome(Data, Remaining, Sent, OutError))
                {
                    return false;
                }
                if (Sent == 0)
                {
                    return true;
                }
                bProgress = true;
                Budget -= Sent;
                LargeFrameOffset += Sent;
                if (LargeFrameOffset == Total)
                {
                    ++FramesSent;
                    LargeFrame = FUBotROSOutgoingFrame();
                    bHasLargeFrame = false;
                    LargeFrameOffset = 0;
                }
                continue;
            }

            if (!RefillSendBuffers())
            {
                // Release a staging buffer that grew while a burst was sent.
                if (Staging.Max() > 4 * StagingTargetBytes)
                {
                    Staging.Empty();
                }
                return true;
            }
        }
        return true;
    }

    bool PumpReceive(bool& bProgress, FString& OutError)
    {
        using namespace UBot::ROSProtocol;

        for (int32 Call = 0; Call < UBot::ROSBridgeSubsystem::Private::ReceiveCallsPerIteration; ++Call)
        {
            int32 BytesRead = 0;
            if (!Socket->Recv(ReceiveBuffer.GetData(), ReceiveBuffer.Num(), BytesRead))
            {
                OutError = TEXT("connection closed by the bridge");
                return false;
            }
            if (BytesRead <= 0)
            {
                return true;
            }

            bProgress = true;
            BytesReceived += BytesRead;
            // Any data counts as life sign, so a large frame that trickles in does not time out.
            LastFrameReceivedSeconds = FPlatformTime::Seconds();
            // Frames completed before a malformed header are still processed (PROTOCOL.md section 2): the
            // parser keeps them in its ready queue, so drain it before reporting the error. A failure of
            // HandleFrame wins, the session ends either way.
            const bool bFeedOk = Parser.Feed(ReceiveBuffer.GetData(), BytesRead);

            FWireFrame Frame;
            while (Parser.PopFrame(Frame))
            {
                ++FramesReceived;
                if (!HandleFrame(MoveTemp(Frame), OutError))
                {
                    return false;
                }
            }

            if (!bFeedOk)
            {
                OutError = FString::Printf(TEXT("malformed frame from the bridge (%s)"), ParseErrorToString(Parser.GetError()));
                return false;
            }
        }
        return true;
    }

    bool HandleFrame(UBot::ROSProtocol::FWireFrame&& Frame, FString& OutError)
    {
        using namespace UBot::ROSProtocol;

        switch (static_cast<EOp>(Frame.Op))
        {
        case EOp::HelloAck:
        {
            FHelloAck Ack;
            if (!ParseHelloAck(Frame.Payload, Ack))
            {
                OutError = TEXT("malformed HELLO_ACK from the bridge");
                return false;
            }
            if (Ack.Protocol != ProtocolVersion)
            {
                OutError = FString::Printf(TEXT("the bridge speaks protocol %d, this client speaks %d"), Ack.Protocol, ProtocolVersion);
                return false;
            }
            if (!bHandshakeDone)
            {
                bHandshakeDone = true;
                PostEvent(EUBotROSBridgeEventType::HandshakeComplete, Ack.Server, Ack.RosDistro);
            }
            return true;
        }
        case EOp::Message:
        {
            if (Frame.Payload.Num() < IdPrefixSize)
            {
                UE_LOG(LogUBotROS, Warning, TEXT("uBot ROS bridge: ignoring a MESSAGE frame without subscription id (%d bytes)."), Frame.Payload.Num());
                return true;
            }
            const int64 Size = Frame.Payload.Num();
            if (PendingIncomingBytes.load() + Size > Config.MaxQueuedBytes)
            {
                ++DroppedIncomingMessages;
                return true;
            }
            FUBotROSBridgeEvent Event;
            Event.Type = EUBotROSBridgeEventType::Message;
            Event.Id = ReadU32LE(Frame.Payload.GetData());
            Event.bHasId = true;
            Event.Payload = MoveTemp(Frame.Payload);
            ++PendingIncomingMessages;
            PendingIncomingBytes += Size;
            Events.Enqueue(MoveTemp(Event));
            return true;
        }
        case EOp::SubCount:
        {
            FUBotROSBridgeEvent Event;
            Event.Type = EUBotROSBridgeEventType::SubscriberCounts;
            if (!ParseSubCount(Frame.Payload, Event.Counts))
            {
                UE_LOG(LogUBotROS, Warning, TEXT("uBot ROS bridge: ignoring a malformed SUB_COUNT frame."));
                return true;
            }
            Events.Enqueue(MoveTemp(Event));
            return true;
        }
        case EOp::Ping:
            EnqueueKeepalive(EOp::Pong);
            return true;
        case EOp::Pong:
            return true;
        case EOp::Error:
        {
            FErrorInfo Info;
            FUBotROSBridgeEvent Event;
            Event.Type = EUBotROSBridgeEventType::BridgeError;
            if (ParseError(Frame.Payload, Info))
            {
                Event.bHasId = Info.Id.IsSet();
                Event.Id = Info.Id.Get(0);
                Event.Detail = Info.Code;
                Event.Text = Info.Message;
            }
            else
            {
                Event.Detail = TEXT("malformed_error");
                Event.Text = FromUtf8(Frame.Payload);
            }
            Events.Enqueue(MoveTemp(Event));
            return true;
        }
        default:
            UE_LOG(LogUBotROS, Warning, TEXT("uBot ROS bridge: ignoring unexpected %s frame from the bridge (%d bytes)."),
                *OpToString(Frame.Op), Frame.Payload.Num());
            return true;
        }
    }

    // Shared with the game thread (Mutex).
    mutable FCriticalSection Mutex;
    bool bWanted = false;
    bool bSessionActive = false;
    TMap<uint32, FRegisteredTopic> PublisherRegistry;
    TMap<uint32, FRegisteredTopic> SubscriptionRegistry;
    TDeque<FUBotROSOutgoingFrame> OrderedQueue;
    TMap<uint32, FUBotROSOutgoingFrame> BestEffortSlots;
    TArray<uint32> BestEffortOrder;
    int64 QueuedBytes = 0;
    uint32 MaxPayloadBytes = UBot::ROSProtocol::DefaultMaxPayloadBytes;
    int64 MaxQueuedBytes = 268435456;
    int64 DroppedMessages = 0;
    int64 RejectedMessages = 0;
    bool bSendingPaused = false;

    // Thread-safe without the mutex.
    TQueue<FUBotROSBridgeEvent, EQueueMode::Mpsc> Events;
    std::atomic<bool> bStopRequested { false };
    std::atomic<int64> FramesSent { 0 };
    std::atomic<int64> FramesReceived { 0 };
    std::atomic<int64> BytesSent { 0 };
    std::atomic<int64> BytesReceived { 0 };
    std::atomic<int64> DroppedIncomingMessages { 0 };
    std::atomic<int32> PendingIncomingMessages { 0 };
    std::atomic<int64> PendingIncomingBytes { 0 };
    std::atomic<int32> SessionCount { 0 };
    FEvent* WakeEvent = nullptr;

    // Game thread (set before the I/O thread starts, read-only while it runs).
    FRunnableThread* Thread = nullptr;
    ISocketSubsystem* SocketSubsystem = nullptr;
    FUBotROSClientConfig Config;

    // I/O thread only.
    FSocket* Socket = nullptr;
    TSharedPtr<FUBotROSResolveState, ESPMode::ThreadSafe> PendingResolve;
    TSharedPtr<FInternetAddr> CachedAddress;
    UBot::ROSProtocol::FFrameParser Parser;
    TArray<uint8> ReceiveBuffer;
    TArray<uint8> Staging;
    int32 StagingOffset = 0;
    int64 StagingFrames = 0;
    int64 StagingPublishFrames = 0;
    FUBotROSOutgoingFrame LargeFrame;
    bool bHasLargeFrame = false;
    int64 LargeFrameOffset = 0;
    bool bHandshakeDone = false;
    double LastFrameReceivedSeconds = 0.0;
    double LastPingSentSeconds = 0.0;
};

// -------------------------------------------------------------------------------------------------
// UUBotROSBridgeSubsystem
// -------------------------------------------------------------------------------------------------

UUBotROSBridgeSubsystem* UUBotROSBridgeSubsystem::Get(const UObject* WorldContext)
{
    if (WorldContext == nullptr || GEngine == nullptr)
    {
        return nullptr;
    }
    const UWorld* World = GEngine->GetWorldFromContextObject(WorldContext, EGetWorldErrorMode::ReturnNull);
    return World != nullptr ? World->GetSubsystem<UUBotROSBridgeSubsystem>() : nullptr;
}

void UUBotROSBridgeSubsystem::Initialize(FSubsystemCollectionBase& Collection)
{
    Super::Initialize(Collection);

    Client = MakeShared<FUBotROSBridgeClient, ESPMode::ThreadSafe>();
    Status = EUBotROSBridgeStatus::Disabled;
}

void UUBotROSBridgeSubsystem::Deinitialize()
{
    // No broadcasts while the world is torn down.
    if (Client.IsValid())
    {
        Client->Shutdown();
        Client->ClearEvents();
        Client.Reset();
    }
    Status = EUBotROSBridgeStatus::Disabled;
    UBot::ROSRuntimeStatus::ClearBridgeStatus(this);
    Publishers.Empty();
    Subscriptions.Empty();
    StaticTransformOwners.Empty();
    MergedStaticTransforms.Empty();
    StaticTransformsMessage.Empty();
    StaticTfPublisherId = 0;
    TfPublisherId = 0;
    ClockPublisherId = 0;

    Super::Deinitialize();
}

void UUBotROSBridgeSubsystem::OnWorldBeginPlay(UWorld& InWorld)
{
    Super::OnWorldBeginPlay(InWorld);

    const UUBotROSSettings* Settings = GetDefault<UUBotROSSettings>();
    if (Settings != nullptr && Settings->bAutoConnect && Client.IsValid() && !Client->IsRunning())
    {
        Connect();
    }
}

void UUBotROSBridgeSubsystem::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);

    ProcessBridgeEvents();
    UpdateStaticTransforms();
    UpdateClock();
}

TStatId UUBotROSBridgeSubsystem::GetStatId() const
{
    RETURN_QUICK_DECLARE_CYCLE_STAT(UUBotROSBridgeSubsystem, STATGROUP_Tickables);
}

bool UUBotROSBridgeSubsystem::DoesSupportWorldType(const EWorldType::Type WorldType) const
{
    return WorldType == EWorldType::Game || WorldType == EWorldType::PIE;
}

// -------------------------------------------------------------------------------------------------
// Topics
// -------------------------------------------------------------------------------------------------

int32 UUBotROSBridgeSubsystem::Advertise(const FString& Topic, const FString& Type, const FUBotROSQos& Qos)
{
    if (!UBot::ROSProtocol::IsValidTopicName(Topic))
    {
        UE_LOG(LogUBotROS, Warning, TEXT("uBot ROS bridge: cannot advertise '%s': not a fully qualified ROS 2 topic name."), *Topic);
        return 0;
    }
    if (!UBot::ROSProtocol::IsValidMessageType(Type))
    {
        UE_LOG(LogUBotROS, Warning, TEXT("uBot ROS bridge: cannot advertise %s: '%s' is not a <package>/msg/<Message> type."), *Topic, *Type);
        return 0;
    }
    if (!Client.IsValid() || NextPublisherId == MAX_int32)
    {
        return 0;
    }

    const int32 Id = NextPublisherId++;
    FPublisherState& State = Publishers.Add(Id);
    State.Topic = Topic;
    State.Type = Type;
    State.Qos = Qos;
    State.SubscriberCount = -1;
    Client->AddPublisher(static_cast<uint32>(Id), Topic, Type, Qos);
    UE_LOG(LogUBotROS, Verbose, TEXT("uBot ROS bridge: publisher %d -> %s (%s)."), Id, *Topic, *Type);
    return Id;
}

void UUBotROSBridgeSubsystem::Unadvertise(int32 PublisherId)
{
    if (Publishers.Remove(PublisherId) == 0)
    {
        return;
    }
    if (Client.IsValid())
    {
        Client->RemovePublisher(static_cast<uint32>(PublisherId));
    }
    if (PublisherId == StaticTfPublisherId)
    {
        StaticTfPublisherId = 0;
    }
    if (PublisherId == TfPublisherId)
    {
        TfPublisherId = 0;
    }
    if (PublisherId == ClockPublisherId)
    {
        ClockPublisherId = 0;
        ClockTopicInUse.Reset();
    }
}

bool UUBotROSBridgeSubsystem::Publish(int32 PublisherId, TArray<uint8>&& Cdr)
{
    const FPublisherState* State = Publishers.Find(PublisherId);
    if (State == nullptr || !Client.IsValid())
    {
        return false;
    }
    if (Cdr.Num() < FUBotCdrWriter::EncapsulationHeaderSize)
    {
        UE_LOG(LogUBotROS, Warning, TEXT("uBot ROS bridge: refusing a %d-byte message on %s (shorter than the CDR encapsulation header)."), Cdr.Num(), *State->Topic);
        return false;
    }

    const int32 Size = Cdr.Num();
    const EUBotROSEnqueueResult Result = Client->EnqueuePublish(static_cast<uint32>(PublisherId), MoveTemp(Cdr));
    switch (Result)
    {
    case EUBotROSEnqueueResult::Queued:
    case EUBotROSEnqueueResult::Dropped:
        return true;
    case EUBotROSEnqueueResult::RejectedOversize:
    case EUBotROSEnqueueResult::RejectedQueueFull:
    {
        const double Now = FPlatformTime::Seconds();
        if (Now - LastRejectWarningSeconds >= UBot::ROSBridgeSubsystem::Private::RejectWarningIntervalSeconds)
        {
            LastRejectWarningSeconds = Now;
            const UUBotROSSettings* Settings = GetDefault<UUBotROSSettings>();
            if (Result == EUBotROSEnqueueResult::RejectedOversize)
            {
                UE_LOG(LogUBotROS, Warning, TEXT("uBot ROS bridge: rejected a %d-byte message on %s: larger than MaxPayloadBytes (%d)."),
                    Size, *State->Topic, Settings->MaxPayloadBytes);
            }
            else
            {
                UE_LOG(LogUBotROS, Warning, TEXT("uBot ROS bridge: rejected a reliable %d-byte message on %s: the outgoing queue is full (MaxQueuedBytes %lld)."),
                    Size, *State->Topic, Settings->MaxQueuedBytes);
            }
        }
        return false;
    }
    case EUBotROSEnqueueResult::UnknownId:
    default:
        return false;
    }
}

int32 UUBotROSBridgeSubsystem::GetSubscriptionCount(int32 PublisherId) const
{
    const FPublisherState* State = Publishers.Find(PublisherId);
    return State != nullptr ? State->SubscriberCount : -1;
}

int32 UUBotROSBridgeSubsystem::Subscribe(const FString& Topic, const FString& Type, const FUBotROSQos& Qos, TFunction<void(TConstArrayView<uint8> Cdr)> OnMessage)
{
    if (!UBot::ROSProtocol::IsValidTopicName(Topic))
    {
        UE_LOG(LogUBotROS, Warning, TEXT("uBot ROS bridge: cannot subscribe to '%s': not a fully qualified ROS 2 topic name."), *Topic);
        return 0;
    }
    if (!UBot::ROSProtocol::IsValidMessageType(Type))
    {
        UE_LOG(LogUBotROS, Warning, TEXT("uBot ROS bridge: cannot subscribe to %s: '%s' is not a <package>/msg/<Message> type."), *Topic, *Type);
        return 0;
    }
    if (!OnMessage)
    {
        UE_LOG(LogUBotROS, Warning, TEXT("uBot ROS bridge: cannot subscribe to %s without a callback."), *Topic);
        return 0;
    }
    if (!Client.IsValid() || NextSubscriptionId == MAX_int32)
    {
        return 0;
    }

    const int32 Id = NextSubscriptionId++;
    FSubscriptionState& State = Subscriptions.Add(Id);
    State.Topic = Topic;
    State.Type = Type;
    State.Qos = Qos;
    State.Callback = MakeShared<TFunction<void(TConstArrayView<uint8>)>>(MoveTemp(OnMessage));
    Client->AddSubscription(static_cast<uint32>(Id), Topic, Type, Qos);
    UE_LOG(LogUBotROS, Verbose, TEXT("uBot ROS bridge: subscription %d <- %s (%s)."), Id, *Topic, *Type);
    return Id;
}

void UUBotROSBridgeSubsystem::Unsubscribe(int32 SubscriptionId)
{
    if (Subscriptions.Remove(SubscriptionId) > 0 && Client.IsValid())
    {
        Client->RemoveSubscription(static_cast<uint32>(SubscriptionId));
    }
}

bool UUBotROSBridgeSubsystem::IsAdvertised(int32 PublisherId) const
{
    return Publishers.Contains(PublisherId);
}

bool UUBotROSBridgeSubsystem::IsSubscribed(int32 SubscriptionId) const
{
    return Subscriptions.Contains(SubscriptionId);
}

// -------------------------------------------------------------------------------------------------
// TF and clock
// -------------------------------------------------------------------------------------------------

void UUBotROSBridgeSubsystem::SetStaticTransforms(const UObject* Owner, const TArray<FUBotRosTransformStamped>& Transforms)
{
    if (Owner == nullptr)
    {
        UE_LOG(LogUBotROS, Warning, TEXT("uBot ROS bridge: SetStaticTransforms needs an owner; %d transform(s) ignored."), Transforms.Num());
        return;
    }
    if (Transforms.Num() == 0)
    {
        ClearStaticTransforms(Owner);
        return;
    }

    FStaticTransformOwner* Entry = StaticTransformOwners.FindByPredicate([Owner](const FStaticTransformOwner& Candidate)
    {
        return Candidate.Owner.Get(/*bEvenIfGarbage*/ true) == Owner;
    });
    if (Entry == nullptr)
    {
        Entry = &StaticTransformOwners.AddDefaulted_GetRef();
        Entry->Owner = Owner;
    }
    Entry->Transforms = Transforms;
    bStaticTransformsDirty = true;
}

void UUBotROSBridgeSubsystem::ClearStaticTransforms(const UObject* Owner)
{
    if (Owner == nullptr)
    {
        return;
    }
    const int32 Removed = StaticTransformOwners.RemoveAll([Owner](const FStaticTransformOwner& Candidate)
    {
        return Candidate.Owner.Get(/*bEvenIfGarbage*/ true) == Owner;
    });
    if (Removed > 0)
    {
        bStaticTransformsDirty = true;
    }
}

void UUBotROSBridgeSubsystem::PublishTransforms(const TArray<FUBotRosTransformStamped>& Transforms)
{
    if (Transforms.Num() == 0)
    {
        return;
    }
    const int32 Id = EnsureInternalPublisher(TfPublisherId, UBot::ROSBridgeSubsystem::Private::TfTopic, FUBotRosTfMessage::TypeName(), FUBotROSQos::Tf());
    if (Id <= 0)
    {
        return;
    }
    FUBotRosTfMessage Message;
    Message.transforms = Transforms;
    Publish(Id, UBot::Ros::Encode(Message));
}

void UUBotROSBridgeSubsystem::UpdateStaticTransforms()
{
    // Owners destroyed without ClearStaticTransforms.
    const int32 Pruned = StaticTransformOwners.RemoveAll([](const FStaticTransformOwner& Entry)
    {
        return !Entry.Owner.IsValid();
    });
    if (Pruned > 0)
    {
        bStaticTransformsDirty = true;
    }

    if (bStaticTransformsDirty)
    {
        bStaticTransformsDirty = false;

        // Frame ids are case sensitive in ROS, so no FString-keyed map here; the sets are small.
        TArray<FUBotRosTransformStamped> Merged;
        for (const FStaticTransformOwner& Entry : StaticTransformOwners)
        {
            for (const FUBotRosTransformStamped& Transform : Entry.Transforms)
            {
                FUBotRosTransformStamped* Existing = Merged.FindByPredicate([&Transform](const FUBotRosTransformStamped& Candidate)
                {
                    return Candidate.child_frame_id.Equals(Transform.child_frame_id, ESearchCase::CaseSensitive);
                });
                if (Existing != nullptr)
                {
                    if (!UBot::ROSBridgeSubsystem::Private::SameTransform(*Existing, Transform))
                    {
                        UE_LOG(LogUBotROS, Warning, TEXT("uBot ROS bridge: static transform of frame '%s' is set by several owners; using the one from %s (parent '%s')."),
                            *Transform.child_frame_id, *GetPathNameSafe(Entry.Owner.Get()), *Transform.header.frame_id);
                    }
                    *Existing = Transform;
                }
                else
                {
                    Merged.Add(Transform);
                }
            }
        }
        MergedStaticTransforms = MoveTemp(Merged);

        if (MergedStaticTransforms.Num() > 0 || StaticTransformsMessage.Num() > 0)
        {
            FUBotRosTfMessage Message;
            Message.transforms = MergedStaticTransforms;
            TArray<uint8> Encoded = UBot::Ros::Encode(Message);
            if (Encoded != StaticTransformsMessage)
            {
                StaticTransformsMessage = MoveTemp(Encoded);
                bStaticTransformsPublishPending = true;
            }
        }
    }

    // Only while connected: the latched message is re-sent after every reconnect anyway.
    if (bStaticTransformsPublishPending && IsBridgeConnected())
    {
        const int32 Id = EnsureInternalPublisher(StaticTfPublisherId, UBot::ROSBridgeSubsystem::Private::TfStaticTopic, FUBotRosTfMessage::TypeName(), FUBotROSQos::Latched());
        if (Id > 0 && Publish(Id, TArray<uint8>(StaticTransformsMessage)))
        {
            bStaticTransformsPublishPending = false;
        }
    }
}

void UUBotROSBridgeSubsystem::UpdateClock()
{
    const UUBotROSSettings* Settings = GetDefault<UUBotROSSettings>();
    const bool bWanted = Settings != nullptr && Settings->bPublishClock;
    if (ClockPublisherId > 0 && (!bWanted || !ClockTopicInUse.Equals(Settings->ClockTopic, ESearchCase::CaseSensitive)))
    {
        Unadvertise(ClockPublisherId);
    }
    if (!bWanted || !IsBridgeConnected())
    {
        return;
    }

    if (ClockPublisherId == 0)
    {
        if (!UBot::ROSProtocol::IsValidTopicName(Settings->ClockTopic))
        {
            if (!bClockTopicWarningLogged)
            {
                UE_LOG(LogUBotROS, Warning, TEXT("uBot ROS bridge: ClockTopic '%s' is not a fully qualified ROS 2 topic name; /clock is not published."), *Settings->ClockTopic);
                bClockTopicWarningLogged = true;
            }
            return;
        }
        ClockPublisherId = Advertise(Settings->ClockTopic, FUBotRosClock::TypeName(), FUBotROSQos(EUBotROSReliability::Reliable, EUBotROSDurability::Volatile, 10));
        if (ClockPublisherId <= 0)
        {
            ClockPublisherId = 0;
            return;
        }
        ClockTopicInUse = Settings->ClockTopic;
        bClockTopicWarningLogged = false;
    }

    FUBotRosClock Message;
    UBot::Ros::SecondsToRosTime(UBot::Clock::GetSimTimeSeconds(GetWorld()), Message.clock.sec, Message.clock.nanosec);
    Publish(ClockPublisherId, UBot::Ros::Encode(Message));
}

int32 UUBotROSBridgeSubsystem::EnsureInternalPublisher(int32& InOutPublisherId, const FString& Topic, const FString& Type, const FUBotROSQos& Qos)
{
    if (InOutPublisherId > 0 && Publishers.Contains(InOutPublisherId))
    {
        return InOutPublisherId;
    }
    InOutPublisherId = Advertise(Topic, Type, Qos);
    return InOutPublisherId;
}

// -------------------------------------------------------------------------------------------------
// Connection
// -------------------------------------------------------------------------------------------------

bool UUBotROSBridgeSubsystem::IsBridgeConnected() const
{
    return Status == EUBotROSBridgeStatus::Connected;
}

EUBotROSBridgeStatus UUBotROSBridgeSubsystem::GetBridgeStatus() const
{
    return Status;
}

void UUBotROSBridgeSubsystem::Connect()
{
    const UUBotROSSettings* Settings = GetDefault<UUBotROSSettings>();
    StartClient(Settings->BridgeHost, Settings->BridgePort);
}

void UUBotROSBridgeSubsystem::ConnectTo(const FString& Host, int32 Port)
{
    StartClient(Host, Port);
}

void UUBotROSBridgeSubsystem::Disconnect()
{
    if (Client.IsValid() && Client->IsRunning())
    {
        UE_LOG(LogUBotROS, Log, TEXT("uBot ROS bridge: disconnecting from %s."), *GetBridgeEndpoint());
    }
    StopClient();
}

FString UUBotROSBridgeSubsystem::GetLastBridgeError() const
{
    return LastError;
}

FString UUBotROSBridgeSubsystem::GetBridgeEndpoint() const
{
    if (!bHasEndpoint)
    {
        return FString();
    }
    // An empty host (an invalid setting) would read as ":8268".
    return FString::Printf(TEXT("%s:%d"), EndpointHost.IsEmpty() ? TEXT("(no host)") : *EndpointHost, EndpointPort);
}

FUBotROSBridgeStats UUBotROSBridgeSubsystem::GetStats() const
{
    FUBotROSBridgeStats Stats;
    if (Client.IsValid())
    {
        Client->FillStats(Stats);
    }
    return Stats;
}

void UUBotROSBridgeSubsystem::SetSendingPaused(bool bPaused)
{
    if (Client.IsValid())
    {
        Client->SetSendingPaused(bPaused);
    }
}

void UUBotROSBridgeSubsystem::StartClient(const FString& Host, int32 Port)
{
    if (!Client.IsValid())
    {
        return;
    }

    const FString TrimmedHost = Host.TrimStartAndEnd();
    if (TrimmedHost.IsEmpty() || Port < 1 || Port > 65535)
    {
        StopClient();

        // The published status and the rosBridgeUnreachable problem name GetBridgeEndpoint(): it must be
        // what was rejected, not the endpoint of an earlier session (or nothing at all). No client runs
        // after StopClient(), so a later valid ConnectTo() is never mistaken for "already connected".
        EndpointHost = TrimmedHost;
        EndpointPort = Port;
        bHasEndpoint = true;

        LastError = FString::Printf(TEXT("invalid bridge endpoint '%s'"), *GetBridgeEndpoint());
        UE_LOG(LogUBotROS, Error, TEXT("uBot ROS bridge: %s."), *LastError);
        SetStatus(EUBotROSBridgeStatus::Error);
        return;
    }
    if (Client->IsRunning() && bHasEndpoint && EndpointHost.Equals(TrimmedHost, ESearchCase::IgnoreCase) && EndpointPort == Port)
    {
        return;
    }

    StopClient();

    const UUBotROSSettings* Settings = GetDefault<UUBotROSSettings>();
    FUBotROSClientConfig Config;
    Config.Host = TrimmedHost;
    Config.Port = Port;
    Config.ReconnectIntervalSeconds = FMath::Max(Settings->ReconnectIntervalSeconds, 0.05);
    Config.MaxPayloadBytes = static_cast<uint32>(FMath::Max(Settings->MaxPayloadBytes, UBot::ROSBridgeSubsystem::Private::MinPayloadBytes));
    Config.MaxQueuedBytes = FMath::Max<int64>(Settings->MaxQueuedBytes, UBot::ROSBridgeSubsystem::Private::MinQueuedBytes);
    // The ClampMin of the setting only applies in the editor UI. A queue smaller than one maximum frame
    // would drop every large message silently (outgoing: it can never fit, incoming: the backlog check
    // fails), so it is raised like the bridge does with max_queued_bytes.
    const int64 MinQueuedForOneFrame = static_cast<int64>(Config.MaxPayloadBytes) + UBot::ROSProtocol::HeaderSize;
    if (Config.MaxQueuedBytes < MinQueuedForOneFrame)
    {
        UE_LOG(LogUBotROS, Warning, TEXT("uBot ROS bridge: MaxQueuedBytes (%lld) is smaller than one maximum frame (MaxPayloadBytes %u + %d header bytes); using %lld."),
            Config.MaxQueuedBytes, Config.MaxPayloadBytes, UBot::ROSProtocol::HeaderSize, MinQueuedForOneFrame);
        Config.MaxQueuedBytes = MinQueuedForOneFrame;
    }
    Config.ClientName = UBot::ROSProtocol::DefaultClientName();

    EndpointHost = TrimmedHost;
    EndpointPort = Port;
    bHasEndpoint = true;
    bLoggedAttemptFailure = false;

    FString Error;
    if (!Client->Start(Config, Error))
    {
        LastError = Error;
        UE_LOG(LogUBotROS, Error, TEXT("uBot ROS bridge: cannot connect to %s: %s."), *GetBridgeEndpoint(), *Error);
        SetStatus(EUBotROSBridgeStatus::Error);
        return;
    }

    UE_LOG(LogUBotROS, Log, TEXT("uBot ROS bridge: connecting to ubot_ros_bridge at %s."), *GetBridgeEndpoint());
    SetStatus(EUBotROSBridgeStatus::Connecting);
}

void UUBotROSBridgeSubsystem::StopClient()
{
    if (Client.IsValid())
    {
        Client->Shutdown();
        // Undelivered messages and connection events of the stopped session are discarded.
        Client->ClearEvents();
    }

    const bool bWasConnected = Status == EUBotROSBridgeStatus::Connected;
    ResetSubscriberCounts();
    SetStatus(EUBotROSBridgeStatus::Disabled);
    if (bWasConnected)
    {
        const FString Reason = TEXT("Disconnect() was called");
        OnBridgeDisconnectedNative.Broadcast(Reason);
        OnBridgeDisconnected.Broadcast(Reason);
    }
}

void UUBotROSBridgeSubsystem::ProcessBridgeEvents()
{
    FUBotROSBridgeEvent Event;
    for (int32 Processed = 0; Processed < UBot::ROSBridgeSubsystem::Private::MaxEventsPerTick; ++Processed)
    {
        // Re-checked every iteration: a callback may disconnect or reconnect.
        if (!Client.IsValid() || !Client->PopEvent(Event))
        {
            break;
        }

        switch (Event.Type)
        {
        case EUBotROSBridgeEventType::AttemptFailed:
        {
            LastError = Event.Text;
            if (!bLoggedAttemptFailure)
            {
                bLoggedAttemptFailure = true;
                UE_LOG(LogUBotROS, Warning, TEXT("uBot ROS bridge: cannot reach ubot_ros_bridge at %s (%s); retrying every %.1f s. Start it with 'ros2 run ubot_ros_bridge bridge'."),
                    *GetBridgeEndpoint(), *Event.Text, GetDefault<UUBotROSSettings>()->ReconnectIntervalSeconds);
            }
            else
            {
                UE_LOG(LogUBotROS, Verbose, TEXT("uBot ROS bridge: connection attempt to %s failed (%s)."), *GetBridgeEndpoint(), *Event.Text);
            }
            if (Status != EUBotROSBridgeStatus::Disabled)
            {
                SetStatus(EUBotROSBridgeStatus::Error);
            }
            break;
        }
        case EUBotROSBridgeEventType::HandshakeComplete:
        {
            bLoggedAttemptFailure = false;
            LastError.Reset();
            SetStatus(EUBotROSBridgeStatus::Connected);
            if (StaticTransformsMessage.Num() > 0)
            {
                // The bridge's latched publisher may be new; re-send the merged /tf_static.
                bStaticTransformsPublishPending = true;
            }
            UE_LOG(LogUBotROS, Log, TEXT("uBot ROS bridge: connected to %s (%s, ROS %s)."),
                *GetBridgeEndpoint(), Event.Text.IsEmpty() ? TEXT("unknown server") : *Event.Text, Event.Detail.IsEmpty() ? TEXT("unknown") : *Event.Detail);
            OnBridgeConnectedNative.Broadcast();
            OnBridgeConnected.Broadcast();
            break;
        }
        case EUBotROSBridgeEventType::SessionEnded:
            HandleSessionEnded(Event.Text);
            break;
        case EUBotROSBridgeEventType::Message:
        {
            const FSubscriptionState* Subscription = Subscriptions.Find(static_cast<int32>(Event.Id));
            if (Subscription == nullptr || Event.Payload.Num() < UBot::ROSProtocol::IdPrefixSize)
            {
                UE_LOG(LogUBotROS, VeryVerbose, TEXT("uBot ROS bridge: dropping a message for unknown subscription %u."), Event.Id);
                break;
            }
            // Keeps the callback alive if it unsubscribes itself.
            const TSharedPtr<TFunction<void(TConstArrayView<uint8>)>> Callback = Subscription->Callback;
            if (Callback.IsValid() && *Callback)
            {
                (*Callback)(TConstArrayView<uint8>(Event.Payload).RightChop(UBot::ROSProtocol::IdPrefixSize));
            }
            break;
        }
        case EUBotROSBridgeEventType::SubscriberCounts:
            for (const TPair<uint32, int32>& Count : Event.Counts)
            {
                if (Count.Key <= static_cast<uint32>(MAX_int32))
                {
                    if (FPublisherState* State = Publishers.Find(static_cast<int32>(Count.Key)))
                    {
                        State->SubscriberCount = Count.Value;
                    }
                }
            }
            break;
        case EUBotROSBridgeEventType::BridgeError:
        {
            FString Target = TEXT("connection");
            if (Event.bHasId)
            {
                const FPublisherState* Publisher = Event.Id <= static_cast<uint32>(MAX_int32) ? Publishers.Find(static_cast<int32>(Event.Id)) : nullptr;
                const FSubscriptionState* Subscription = Event.Id <= static_cast<uint32>(MAX_int32) ? Subscriptions.Find(static_cast<int32>(Event.Id)) : nullptr;
                Target = FString::Printf(TEXT("id %u"), Event.Id);
                if (Publisher != nullptr)
                {
                    Target += FString::Printf(TEXT(" (publisher %s)"), *Publisher->Topic);
                }
                if (Subscription != nullptr)
                {
                    Target += FString::Printf(TEXT(" (subscription %s)"), *Subscription->Topic);
                }
            }
            UE_LOG(LogUBotROS, Warning, TEXT("uBot ROS bridge: the bridge reported '%s' for %s: %s"), *Event.Detail, *Target, *Event.Text);
            break;
        }
        default:
            break;
        }
    }
}

void UUBotROSBridgeSubsystem::HandleSessionEnded(const FString& Reason)
{
    const bool bWasConnected = Status == EUBotROSBridgeStatus::Connected;
    ResetSubscriberCounts();
    LastError = Reason;
    if (Status != EUBotROSBridgeStatus::Disabled)
    {
        SetStatus(EUBotROSBridgeStatus::Error);
    }

    if (bWasConnected)
    {
        UE_LOG(LogUBotROS, Warning, TEXT("uBot ROS bridge: lost the connection to %s (%s); reconnecting every %.1f s."),
            *GetBridgeEndpoint(), *Reason, GetDefault<UUBotROSSettings>()->ReconnectIntervalSeconds);
        OnBridgeDisconnectedNative.Broadcast(Reason);
        OnBridgeDisconnected.Broadcast(Reason);
    }
    else if (!bLoggedAttemptFailure)
    {
        bLoggedAttemptFailure = true;
        UE_LOG(LogUBotROS, Warning, TEXT("uBot ROS bridge: the connection to %s ended before the handshake (%s); retrying."), *GetBridgeEndpoint(), *Reason);
    }
}

void UUBotROSBridgeSubsystem::SetStatus(EUBotROSBridgeStatus NewStatus)
{
    Status = NewStatus;
    UBot::ROSRuntimeStatus::PublishBridgeStatus(this, Status, GetBridgeEndpoint());
}

void UUBotROSBridgeSubsystem::ResetSubscriberCounts()
{
    for (TPair<int32, FPublisherState>& Entry : Publishers)
    {
        Entry.Value.SubscriberCount = -1;
    }
}
