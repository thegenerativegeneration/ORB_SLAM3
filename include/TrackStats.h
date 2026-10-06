#ifndef TRACKSTATS_H
#define TRACKSTATS_H

#include <chrono>
#include <time.h>

namespace ORB_SLAM3
{

/// Milliseconds on a steady clock; only differences mean anything.
inline double WallMs()
{
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

/// The calling thread's CPU time in milliseconds; only differences mean anything.
inline double ThreadCpuMs()
{
    timespec ts;
    clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ts);
    return ts.tv_sec * 1e3 + ts.tv_nsec * 1e-6;
}

/// Cost of the last Tracking::GrabImageRGBD, recorded for every frame (a dozen clock reads, no allocation).
struct TrackTiming
{
    double totalMs = 0;     ///< wall time of the whole call
    double cpuMs = 0;       ///< the tracking thread's CPU time over the same span
    double extractMs = 0;   ///< frame construction: grey and depth conversion, ORB extraction, keypoint depth, grid
    double imuMs = 0;       ///< PreintegrateIMU
    double lockWaitMs = 0;  ///< waiting for the active map's mMutexMapUpdate
    double predictMs = 0;   ///< first pose estimate: motion model, reference keyframe or relocalisation
    double localMapMs = 0;  ///< TrackLocalMap and the IMU bookkeeping after it
    double keyFrameMs = 0;  ///< NeedNewKeyFrame and CreateNewKeyFrame
    int localKeyFrames = 0; ///< local keyframes after this frame
    int localMapPoints = 0; ///< local map points after this frame
};

/// LocalMapping's work since the last LocalMapping::TakeStats.
struct MappingStats
{
    int keyFrames = 0;    ///< keyframes processed
    double wallMs = 0;    ///< their wall time, from ProcessNewKeyFrame to the hand-over to LoopClosing, summed
    double cpuMs = 0;     ///< the LocalMapping thread's CPU time over the same spans, summed
    double maxWallMs = 0; ///< the longest one
    int abortedBA = 0;    ///< local bundle adjustments with an abort request pending when they returned: a newer
                          ///< keyframe (also one queued just after the BA finished), Tracking's InterruptBA, or a
                          ///< stop request from LoopClosing or a reset
};

} // namespace ORB_SLAM3

#endif // TRACKSTATS_H
