#ifndef TRACKWATCH_H
#define TRACKWATCH_H

#include <atomic>

namespace ORB_SLAM3
{

/// Where the tracking thread is inside System::TrackRGBD, for a watchdog on another thread that must not take
/// ORB-SLAM3's locks. Process-wide (one System per process). The tracking thread writes it with relaxed stores; any
/// thread may read it.
enum class TrackPhase : int
{
    Idle = 0,             ///< not in TrackRGBD, or returned
    ResetCheck = 1,       ///< System: taking mMutexReset for the deferred-reset check, or the check under it
    Reset = 2,            ///< Tracking::Reset, outside the waits below
    ResetActiveMap = 3,   ///< Tracking::ResetActiveMap, outside the waits below
    WaitLocalMapping = 4, ///< LocalMapping::RequestReset or RequestResetActiveMap: polling until LocalMapping resets
    WaitLoopClosing = 5,  ///< LoopClosing::RequestReset or RequestResetActiveMap: polling until LoopClosing resets
    Frame = 6,            ///< IMU hand-over, frame construction (ORB extraction), Track up to the map lock
    MapLock = 7,          ///< Tracking::Track: waiting for the active map's mMutexMapUpdate
    Initialization = 8,   ///< StereoInitialization
    Tracking = 9,         ///< Tracking::Track under the map lock, outside initialisation
};

inline std::atomic<int> gTrackPhase{0};

inline void SetTrackPhase(TrackPhase phase)
{
    gTrackPhase.store(static_cast<int>(phase), std::memory_order_relaxed);
}

inline TrackPhase GetTrackPhase()
{
    return static_cast<TrackPhase>(gTrackPhase.load(std::memory_order_relaxed));
}

/// LocalMapping's flags that decide whether it answers a reset request, read without its locks.
struct LocalMappingWatch
{
    bool stopped = false;                 ///< in its stopped loop (or finished): no reset is answered there
    bool stopRequested = false;
    bool resetRequested = false;          ///< full reset pending
    bool resetActiveMapRequested = false;
    bool finished = false;
    bool processingKeyFrame = false;      ///< between taking a keyframe and handing it to LoopClosing
};

/// LoopClosing's flags that decide whether it answers a reset request, read without its locks.
struct LoopClosingWatch
{
    bool resetRequested = false;
    bool resetActiveMapRequested = false;
    bool finished = false;
    bool processingKeyFrame = false;      ///< a keyframe taken from the queue is in loop or merge detection/correction
    bool runningGBA = false;
};

/// System::GetTrackWatch: the tracking phase and the back-end flags, each read on its own (not one snapshot).
struct TrackWatch
{
    TrackPhase phase = TrackPhase::Idle;
    LocalMappingWatch localMapping;
    LoopClosingWatch loopClosing;
};

} // namespace ORB_SLAM3

#endif // TRACKWATCH_H
