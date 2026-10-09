#pragma once

#include "CoreMinimal.h"
#include "Containers/ArrayView.h"
#include "Subsystems/WorldSubsystem.h"
#include "Templates/Function.h"
#include "Templates/SharedPointer.h"
#include "UObject/WeakObjectPtrTemplates.h"
#include "UBotROSMessageTypes.h"
#include "UBotROSTypes.h"
#include "UBotROSBridgeSubsystem.generated.h"

class FUBotROSBridgeClient;

DECLARE_DYNAMIC_MULTICAST_DELEGATE(FUBotROSBridgeConnectedSignature);
DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FUBotROSBridgeDisconnectedSignature, const FString&, Reason);

DECLARE_MULTICAST_DELEGATE(FUBotROSBridgeConnectedNative);
DECLARE_MULTICAST_DELEGATE_OneParam(FUBotROSBridgeDisconnectedNative, const FString& /*Reason*/);

/** Counters of one world's bridge connection (cumulative since the subsystem was created). */
struct UBOTROS_API FUBotROSBridgeStats
{
    /** Frames completely written to the socket. */
    int64 FramesSent = 0;
    /** Complete frames received from the bridge. */
    int64 FramesReceived = 0;
    int64 BytesSent = 0;
    int64 BytesReceived = 0;
    /**
     * Outgoing messages that were dropped by policy: best_effort messages replaced by a newer one or
     * published while no session was active, data published while the bridge is Disabled, data
     * evicted to make room for reliable messages, and frames in flight when a connection was lost.
     */
    int64 DroppedMessages = 0;
    /** Outgoing messages refused by Publish (reliable queue full or payload above MaxPayloadBytes). */
    int64 RejectedMessages = 0;
    /** Received MESSAGE frames dropped because the dispatch backlog exceeded MaxQueuedBytes. */
    int64 DroppedIncomingMessages = 0;
    /** Bytes of frames waiting in the outgoing queues (not yet handed to the socket). */
    int64 QueuedBytes = 0;
    /** Received MESSAGE frames waiting for dispatch on the game thread. */
    int32 PendingIncomingMessages = 0;
    /** TCP connections established (each one starts a new session with HELLO). */
    int32 SessionCount = 0;
};

/**
 * One TCP connection per game/PIE world to the ROS-side ubot_ros_bridge node (wire protocol v1,
 * docs/PROTOCOL.md). Socket I/O runs on a background thread; the game thread never blocks on the
 * socket. Received messages are dispatched to subscription callbacks on the game thread during Tick.
 *
 * Outgoing queue policy: a best_effort publisher keeps only its newest pending message (older ones
 * are counted as dropped) and drops data while no session is active; reliable messages are queued in
 * order, also while the bridge is reconnecting, up to MaxQueuedBytes (then the oldest pending
 * best_effort data is evicted first, then new reliable data is rejected with a warning). While the
 * bridge is Disabled nothing is queued. Advertisements and subscriptions persist across reconnects
 * and are re-sent (with the same ids) after every HELLO.
 *
 * All methods are game thread only.
 */
UCLASS()
class UBOTROS_API UUBotROSBridgeSubsystem : public UTickableWorldSubsystem
{
    GENERATED_BODY()

public:
    /** Bridge subsystem of WorldContext's world; nullptr when there is no world or it is not a game/PIE world. */
    static UUBotROSBridgeSubsystem* Get(const UObject* WorldContext);

    //~ Begin USubsystem interface
    virtual void Initialize(FSubsystemCollectionBase& Collection) override;
    virtual void Deinitialize() override;
    //~ End USubsystem interface

    //~ Begin UWorldSubsystem interface
    virtual void OnWorldBeginPlay(UWorld& InWorld) override;
    //~ End UWorldSubsystem interface

    //~ Begin FTickableGameObject interface
    virtual void Tick(float DeltaTime) override;
    virtual TStatId GetStatId() const override;
    // Connection events and received messages are processed while the game is paused too.
    virtual bool IsTickableWhenPaused() const override { return true; }
    //~ End FTickableGameObject interface

    // ---------------------------------------------------------------------------------------------
    // Topics (C++)
    // ---------------------------------------------------------------------------------------------

    /**
     * Registers a publisher. Topic must be a fully qualified ROS 2 name ("/ns/topic") and Type a
     * "<package>/msg/<Message>" name. Returns the publisher id (> 0, never reused by this subsystem),
     * or 0 when the topic or type is invalid. The ADVERTISE is sent now if a session is active and
     * after every (re)connect otherwise.
     */
    int32 Advertise(const FString& Topic, const FString& Type, const FUBotROSQos& Qos);

    /** Removes a publisher; its pending best_effort message is discarded. Unknown ids are ignored. */
    void Unadvertise(int32 PublisherId);

    /**
     * Queues one serialized message (CDR incl. the 4-byte encapsulation header) for PublisherId.
     * Returns false if the id is unknown or the message was refused (payload above MaxPayloadBytes,
     * reliable queue full). Returns true when the message was queued or dropped by QoS policy
     * (best_effort while not connected or replaced by a newer message, anything while Disabled).
     */
    bool Publish(int32 PublisherId, TArray<uint8>&& Cdr);

    /** Number of ROS subscribers the bridge last reported for the publisher's topic; -1 = unknown (not reported yet, unknown id, or not connected). */
    int32 GetSubscriptionCount(int32 PublisherId) const;

    /**
     * Subscribes to Topic. OnMessage receives each message's CDR bytes (incl. encapsulation header) on
     * the game thread during Tick; the view is only valid during the call. Returns the subscription id
     * (> 0, never reused), or 0 for an invalid topic/type or an unbound callback.
     */
    int32 Subscribe(const FString& Topic, const FString& Type, const FUBotROSQos& Qos, TFunction<void(TConstArrayView<uint8> Cdr)> OnMessage);

    /** Removes a subscription; messages already received for it are discarded. Safe inside its own callback. */
    void Unsubscribe(int32 SubscriptionId);

    /** True while PublisherId is advertised by this subsystem. */
    bool IsAdvertised(int32 PublisherId) const;

    /** True while SubscriptionId is registered with this subsystem. */
    bool IsSubscribed(int32 SubscriptionId) const;

    // ---------------------------------------------------------------------------------------------
    // TF and clock (C++)
    // ---------------------------------------------------------------------------------------------

    /**
     * Static TF aggregation: every owner's static transforms are merged into ONE latched /tf_static
     * message (tf2_msgs/msg/TFMessage, reliable / transient_local / depth 1), re-published on the next
     * Tick whenever the merged set changes and after every reconnect (only while connected; nothing is
     * queued meanwhile). Owners are merged in the order they registered (first call after a clear); a
     * child frame set by several owners keeps the transform of the owner registered last (with a
     * warning when they differ). Replaces Owner's previous set; an empty array is the same as
     * ClearStaticTransforms. Owners that are destroyed without clearing are pruned on the next Tick.
     */
    void SetStaticTransforms(const UObject* Owner, const TArray<FUBotRosTransformStamped>& Transforms);

    /** Removes Owner's static transforms from the merged /tf_static set. */
    void ClearStaticTransforms(const UObject* Owner);

    /** Publishes dynamic transforms on /tf (reliable, volatile, depth 100). Empty arrays are ignored. */
    void PublishTransforms(const TArray<FUBotRosTransformStamped>& Transforms);

    /** The merged /tf_static set as it was last built (game thread). */
    const TArray<FUBotRosTransformStamped>& GetMergedStaticTransforms() const { return MergedStaticTransforms; }

    // ---------------------------------------------------------------------------------------------
    // Connection (Blueprint)
    // ---------------------------------------------------------------------------------------------

    /** True once the bridge has answered HELLO with a compatible HELLO_ACK. */
    UFUNCTION(BlueprintPure, Category = "uBot|ROS")
    bool IsBridgeConnected() const;

    /**
     * Disabled: not connecting (Disconnect() called, or auto-connect off and Connect() not called).
     * Connecting: first attempt in progress / waiting for HELLO_ACK. Connected: handshake done.
     * Error: the last attempt failed or the connection was lost; retrying every ReconnectIntervalSeconds.
     */
    UFUNCTION(BlueprintPure, Category = "uBot|ROS")
    EUBotROSBridgeStatus GetBridgeStatus() const;

    /** Connects to BridgeHost:BridgePort from the project settings (no-op if already connecting there). */
    UFUNCTION(BlueprintCallable, Category = "uBot|ROS")
    void Connect();

    /** Connects to an explicit endpoint, overriding the project settings for this world. */
    UFUNCTION(BlueprintCallable, Category = "uBot|ROS")
    void ConnectTo(const FString& Host, int32 Port);

    /** Closes the connection and stops reconnecting; pending outgoing data is discarded. */
    UFUNCTION(BlueprintCallable, Category = "uBot|ROS")
    void Disconnect();

    /** Reason of the last failed attempt, lost connection or protocol error; empty when none. */
    UFUNCTION(BlueprintPure, Category = "uBot|ROS")
    FString GetLastBridgeError() const;

    /** "host:port" this subsystem connects (or last connected) to; empty when never connected. */
    UFUNCTION(BlueprintPure, Category = "uBot|ROS")
    FString GetBridgeEndpoint() const;

    /** Fired when the handshake with the bridge completes (after every reconnect). */
    UPROPERTY(BlueprintAssignable, Category = "uBot|ROS")
    FUBotROSBridgeConnectedSignature OnBridgeConnected;

    /** Fired when an established connection ends (lost, protocol error or Disconnect()). */
    UPROPERTY(BlueprintAssignable, Category = "uBot|ROS")
    FUBotROSBridgeDisconnectedSignature OnBridgeDisconnected;

    /** Native counterparts of OnBridgeConnected / OnBridgeDisconnected, broadcast right before them. */
    FUBotROSBridgeConnectedNative OnBridgeConnectedNative;
    FUBotROSBridgeDisconnectedNative OnBridgeDisconnectedNative;

    // ---------------------------------------------------------------------------------------------
    // Diagnostics (C++)
    // ---------------------------------------------------------------------------------------------

    FUBotROSBridgeStats GetStats() const;

    /**
     * While paused, frames queued after this call are kept in the queues (and keep being replaced /
     * limited per QoS) instead of being written to the socket. Simulates a congested link; used by tests.
     */
    void SetSendingPaused(bool bPaused);

protected:
    //~ Begin UWorldSubsystem interface
    virtual bool DoesSupportWorldType(const EWorldType::Type WorldType) const override;
    //~ End UWorldSubsystem interface

private:
    struct FPublisherState
    {
        FString Topic;
        FString Type;
        FUBotROSQos Qos;
        int32 SubscriberCount = -1;
    };

    struct FSubscriptionState
    {
        FString Topic;
        FString Type;
        FUBotROSQos Qos;
        TSharedPtr<TFunction<void(TConstArrayView<uint8>)>> Callback;
    };

    struct FStaticTransformOwner
    {
        TWeakObjectPtr<const UObject> Owner;
        TArray<FUBotRosTransformStamped> Transforms;
    };

    void StartClient(const FString& Host, int32 Port);
    void StopClient();
    void ProcessBridgeEvents();
    void HandleSessionEnded(const FString& Reason);
    void SetStatus(EUBotROSBridgeStatus NewStatus);
    void ResetSubscriberCounts();
    void UpdateStaticTransforms();
    void UpdateClock();
    int32 EnsureInternalPublisher(int32& InOutPublisherId, const FString& Topic, const FString& Type, const FUBotROSQos& Qos);

    TSharedPtr<FUBotROSBridgeClient, ESPMode::ThreadSafe> Client;

    TMap<int32, FPublisherState> Publishers;
    TMap<int32, FSubscriptionState> Subscriptions;
    int32 NextPublisherId = 1;
    int32 NextSubscriptionId = 1;

    EUBotROSBridgeStatus Status = EUBotROSBridgeStatus::Disabled;
    FString LastError;
    FString EndpointHost;
    int32 EndpointPort = 0;
    bool bHasEndpoint = false;
    bool bLoggedAttemptFailure = false;
    double LastRejectWarningSeconds = -1.0e9;

    TArray<FStaticTransformOwner> StaticTransformOwners;
    TArray<FUBotRosTransformStamped> MergedStaticTransforms;
    /** Encoded TFMessage of MergedStaticTransforms as last built; empty until the first non-empty set. */
    TArray<uint8> StaticTransformsMessage;
    bool bStaticTransformsDirty = false;
    bool bStaticTransformsPublishPending = false;
    int32 StaticTfPublisherId = 0;
    int32 TfPublisherId = 0;
    int32 ClockPublisherId = 0;
    FString ClockTopicInUse;
    bool bClockTopicWarningLogged = false;
};
