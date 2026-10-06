/**
 * Offline RGB-D-inertial driver for EuRoC-style folders with a registered depth stream.
 *
 *   <seq>/mav0/cam0/data.csv   + cam0/data/<t_ns>.png    (8-bit gray or colour)
 *   <seq>/mav0/depth0/data.csv + depth0/data/<t_ns>.png  (16-bit, depth units per RGBD.DepthMapFactor,
 *                                                          0 = invalid, registered to cam0 pixels)
 *   <seq>/mav0/imu0/data.csv   (t_ns, wx, wy, wz [rad/s], ax, ay, az [m/s^2])
 *
 * cam0 frames without a depth image are NOT fed to the tracker. Their IMU samples are carried over
 * to the next fed frame, so preintegration simply spans a longer interval. Such frames appear in
 * <out_prefix>_tracking.csv with state -2 (SKIPPED_NO_DEPTH).
 *
 * Modelled on Examples/Monocular-Inertial/mono_inertial_euroc.cc and Examples/RGB-D/rgbd_tum.cc.
 */

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <System.h>
#include "ImuTypes.h"

using namespace std;

namespace {

struct CsvFrame {
    int64_t t_ns;
    string file;  // absolute path
};

struct ImuSample {
    int64_t t_ns;
    double wx, wy, wz, ax, ay, az;
};

string Trim(const string &s)
{
    const size_t b = s.find_first_not_of(" \t\r\n");
    if (b == string::npos) return "";
    const size_t e = s.find_last_not_of(" \t\r\n");
    return s.substr(b, e - b + 1);
}

vector<string> SplitCsv(const string &line)
{
    vector<string> out;
    stringstream ss(line);
    string item;
    while (getline(ss, item, ',')) out.push_back(Trim(item));
    return out;
}

// EuRoC image list: "#timestamp [ns],filename". Falls back to "<t_ns>.png" if the filename column is missing.
bool LoadFrameCsv(const string &csvPath, const string &dataDir, vector<CsvFrame> &frames)
{
    ifstream f(csvPath);
    if (!f.is_open()) return false;
    string line;
    while (getline(f, line)) {
        line = Trim(line);
        if (line.empty() || line[0] == '#') continue;
        const vector<string> cols = SplitCsv(line);
        if (cols.empty()) continue;
        CsvFrame fr;
        fr.t_ns = stoll(cols[0]);
        const string name = (cols.size() > 1 && !cols[1].empty()) ? cols[1] : cols[0] + ".png";
        fr.file = dataDir + "/" + name;
        frames.push_back(fr);
    }
    sort(frames.begin(), frames.end(), [](const CsvFrame &a, const CsvFrame &b) { return a.t_ns < b.t_ns; });
    return true;
}

bool LoadImuCsv(const string &csvPath, vector<ImuSample> &imu)
{
    ifstream f(csvPath);
    if (!f.is_open()) return false;
    string line;
    while (getline(f, line)) {
        line = Trim(line);
        if (line.empty() || line[0] == '#') continue;
        const vector<string> c = SplitCsv(line);
        if (c.size() < 7) continue;
        imu.push_back({stoll(c[0]), stod(c[1]), stod(c[2]), stod(c[3]), stod(c[4]), stod(c[5]), stod(c[6])});
    }
    sort(imu.begin(), imu.end(), [](const ImuSample &a, const ImuSample &b) { return a.t_ns < b.t_ns; });
    return true;
}

const char *StateName(int s)
{
    switch (s) {
        case -2: return "SKIPPED_NO_DEPTH";
        case -1: return "SYSTEM_NOT_READY";
        case 0: return "NO_IMAGES_YET";
        case 1: return "NOT_INITIALIZED";
        case 2: return "OK";
        case 3: return "RECENTLY_LOST";
        case 4: return "LOST";
        case 5: return "OK_KLT";
        default: return "UNKNOWN";
    }
}

ORB_SLAM3::IMU::Point ToOrb(const ImuSample &m)
{
    return ORB_SLAM3::IMU::Point(m.ax, m.ay, m.az, m.wx, m.wy, m.wz, m.t_ns * 1e-9);
}

void Usage()
{
    cerr << "Usage: rgbd_inertial_euroc_folder <vocab> <settings.yaml> <seq_dir> <out_prefix> [--no-viewer] [--viewer] [--no-pace]\n"
            "  --no-viewer  run without Pangolin viewer (default; the viewer thread crashes on macOS)\n"
            "  --viewer     enable the viewer (Linux only in practice)\n"
            "  --no-pace    feed frames as fast as possible instead of at the recorded frame rate\n"
            "Writes <out_prefix>_f.txt, <out_prefix>_kf.txt (EuRoC format, IMU/body frame) and <out_prefix>_tracking.csv\n";
}

}  // namespace

int main(int argc, char **argv)
{
    if (argc < 5) {
        Usage();
        return 1;
    }
    const string vocPath = argv[1], settingsPath = argv[2], seqDir = argv[3], outPrefix = argv[4];
    bool useViewer = false, pace = true;
    for (int i = 5; i < argc; ++i) {
        const string a = argv[i];
        if (a == "--no-viewer") useViewer = false;
        else if (a == "--viewer") useViewer = true;
        else if (a == "--no-pace") pace = false;
        else {
            cerr << "Unknown option " << a << "\n";
            Usage();
            return 1;
        }
    }

    const string mav = seqDir + "/mav0";
    vector<CsvFrame> camFrames, depthFrames;
    vector<ImuSample> imu;
    if (!LoadFrameCsv(mav + "/cam0/data.csv", mav + "/cam0/data", camFrames) || camFrames.empty()) {
        cerr << "ERROR: no cam0 frames in " << mav << "/cam0/data.csv\n";
        return 1;
    }
    if (!LoadFrameCsv(mav + "/depth0/data.csv", mav + "/depth0/data", depthFrames)) {
        cerr << "ERROR: cannot read " << mav << "/depth0/data.csv\n";
        return 1;
    }
    if (!LoadImuCsv(mav + "/imu0/data.csv", imu) || imu.empty()) {
        cerr << "ERROR: no IMU samples in " << mav << "/imu0/data.csv\n";
        return 1;
    }
    map<int64_t, string> depthByT;
    for (const CsvFrame &d : depthFrames) depthByT[d.t_ns] = d.file;
    cout << "cam0 frames: " << camFrames.size() << ", depth frames: " << depthFrames.size()
         << ", IMU samples: " << imu.size() << endl;

    ofstream fTrack(outPrefix + "_tracking.csv");
    if (!fTrack.is_open()) {
        cerr << "ERROR: cannot write " << outPrefix << "_tracking.csv\n";
        return 1;
    }
    fTrack << "t_ns,state,state_name,has_depth,n_imu,n_tracked_mappoints\n";

    ORB_SLAM3::System SLAM(vocPath, settingsPath, ORB_SLAM3::System::IMU_RGBD, useViewer);
    const float imageScale = SLAM.GetImageScale();

    size_t imuIdx = 0;
    vector<ORB_SLAM3::IMU::Point> pendingImu;  // samples since the last fed frame
    bool fedAny = false;
    size_t nFed = 0, nOk = 0;
    double trackTotal = 0;

    for (size_t ni = 0; ni < camFrames.size(); ++ni) {
        const CsvFrame &fr = camFrames[ni];
        const double tframe = fr.t_ns * 1e-9;

        while (imuIdx < imu.size() && imu[imuIdx].t_ns <= fr.t_ns) pendingImu.push_back(ToOrb(imu[imuIdx++]));

        cv::Mat depth;
        auto it = depthByT.find(fr.t_ns);
        if (it != depthByT.end()) depth = cv::imread(it->second, cv::IMREAD_UNCHANGED);

        int state = -2;
        size_t nImuFed = 0, nMp = 0;
        double ttrack = 0;
        if (!depth.empty()) {
            cv::Mat im = cv::imread(fr.file, cv::IMREAD_UNCHANGED);
            if (im.empty()) {
                cerr << "ERROR: failed to load image " << fr.file << endl;
                return 1;
            }
            if (depth.type() != CV_16UC1 || depth.size() != im.size()) {
                cerr << "WARN: depth " << it->second << " is not 16-bit or not the image size; frame skipped" << endl;
                depth.release();
            } else {
                if (imageScale != 1.f) {
                    const cv::Size sz(im.cols * imageScale, im.rows * imageScale);
                    cv::resize(im, im, sz);
                    cv::resize(depth, depth, sz, 0, 0, cv::INTER_NEAREST);
                }
                // Like mono_inertial_euroc: no IMU for the first frame; the next interval starts at the
                // last sample at or before it.
                vector<ORB_SLAM3::IMU::Point> feedImu;
                if (fedAny) feedImu.swap(pendingImu);
                else if (!pendingImu.empty()) pendingImu.erase(pendingImu.begin(), pendingImu.end() - 1);
                nImuFed = feedImu.size();

                const auto t1 = chrono::steady_clock::now();
                SLAM.TrackRGBD(im, depth, tframe, feedImu, fr.file);
                const auto t2 = chrono::steady_clock::now();
                ttrack = chrono::duration_cast<chrono::duration<double>>(t2 - t1).count();
                trackTotal += ttrack;

                fedAny = true;
                ++nFed;
                state = SLAM.GetTrackingState();
                if (state == 2 || state == 5) ++nOk;
                for (ORB_SLAM3::MapPoint *mp : SLAM.GetTrackedMapPoints())
                    if (mp) ++nMp;
            }
        }
        fTrack << fr.t_ns << ',' << state << ',' << StateName(state) << ',' << (state == -2 ? 0 : 1) << ','
               << nImuFed << ',' << nMp << '\n';

        if (pace && state != -2) {
            double T = 0;
            if (ni + 1 < camFrames.size()) T = (camFrames[ni + 1].t_ns - fr.t_ns) * 1e-9;
            else if (ni > 0) T = (fr.t_ns - camFrames[ni - 1].t_ns) * 1e-9;
            if (ttrack < T) this_thread::sleep_for(chrono::duration<double>(T - ttrack));
        }
    }
    fTrack.close();

    SLAM.Shutdown();

    SLAM.SaveTrajectoryEuRoC(outPrefix + "_f.txt");
    SLAM.SaveKeyFrameTrajectoryEuRoC(outPrefix + "_kf.txt");

    cout << "\nframes: " << camFrames.size() << ", fed (with depth): " << nFed << ", tracked OK: " << nOk;
    if (nFed) cout << " (" << 100.0 * nOk / nFed << "% of fed), mean track time " << 1e3 * trackTotal / nFed << " ms";
    cout << endl;
    return 0;
}
