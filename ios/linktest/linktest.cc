// Link test only: references the API the probe app needs. Never meant to run.
#include <System.h>
#include <ImuTypes.h>
#include <opencv2/core.hpp>
#include <cstdio>

int main(int argc, char** argv)
{
    if (argc < 3) return 0;
    ORB_SLAM3::System slam(argv[1], argv[2], ORB_SLAM3::System::IMU_RGBD, false);
    std::vector<ORB_SLAM3::IMU::Point> imu;
    imu.emplace_back(0.f, 0.f, 9.81f, 0.f, 0.f, 0.f, 0.0);
    cv::Mat im(480, 640, CV_8UC1, cv::Scalar(0)), depth(480, 640, CV_16UC1, cv::Scalar(0));
    Sophus::SE3f Tcw = slam.TrackRGBD(im, depth, 0.0, imu);
    const int state = slam.GetTrackingState();
    const bool lost = slam.isLost();
    std::printf("%d %d %f\n", state, (int)lost, Tcw.translation().x());
    slam.Shutdown();
    return 0;
}
