/**
 * Capture-cycle driver: one IMU_RGBD System used for several captures, as the capture-guide iOS app does.
 *
 *   rgbd_inertial_capture_cycle <vocab> <settings.yaml> <seq_dir> <captures> [--black i,j,...]
 *     captures  comma-separated frame index ranges "a-b" (b exclusive) of the EuRoC-style folder (layout as in
 *               rgbd_inertial_euroc_folder.cc), e.g. "0-300,400-600"; System::Reset() runs between two captures
 *     --black   frame indices replaced by a uniform dark image (no ORB keypoints)
 *
 * The first frame of each capture gets no IMU samples (the app's queue starts empty), later frames the samples in
 * (previous frame, frame]. Frames without a depth image are skipped. Exits 0 after the last capture; prints one line
 * per frame with the tracking state to stderr.
 */
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <System.h>
#include "ImuTypes.h"

using namespace std;

struct Row { int64_t t; string file; };
struct Imu { int64_t t; double wx, wy, wz, ax, ay, az; };

static vector<Row> loadFrames(const string &dir) {
    vector<Row> out;
    ifstream f(dir + "/data.csv");
    string line;
    while (getline(f, line)) {
        if (line.empty() || line[0] == '#') continue;
        auto c = line.find(',');
        int64_t t = stoll(line.substr(0, c));
        string name = line.substr(c + 1);
        while (!name.empty() && (name.back() == '\r' || name.back() == ' ')) name.pop_back();
        out.push_back({t, dir + "/data/" + name});
    }
    return out;
}

static vector<Imu> loadImu(const string &path) {
    vector<Imu> out;
    ifstream f(path);
    string line;
    while (getline(f, line)) {
        if (line.empty() || line[0] == '#') continue;
        Imu m;
        char comma;
        stringstream ss(line);
        ss >> m.t >> comma >> m.wx >> comma >> m.wy >> comma >> m.wz >> comma >> m.ax >> comma >> m.ay >> comma >> m.az;
        out.push_back(m);
    }
    return out;
}

int main(int argc, char **argv) {
    if (argc < 5) {
        cerr << "usage: rgbd_inertial_capture_cycle <voc> <settings> <seq_dir> <a-b,c-d,...> [--black i,j,...]\n";
        return 2;
    }
    const string mav = string(argv[3]) + "/mav0";
    vector<pair<int, int>> captures;
    {
        stringstream ss(argv[4]);
        string r;
        while (getline(ss, r, ',')) {
            auto d = r.find('-');
            captures.push_back({stoi(r.substr(0, d)), stoi(r.substr(d + 1))});
        }
    }
    set<int> black;
    for (int i = 5; i + 1 < argc; ++i) {
        if (string(argv[i]) == "--black") {
            stringstream ss(argv[i + 1]);
            string v;
            while (getline(ss, v, ',')) black.insert(stoi(v));
        }
    }
    vector<Row> cam = loadFrames(mav + "/cam0"), depthRows = loadFrames(mav + "/depth0");
    vector<Imu> imu = loadImu(mav + "/imu0/data.csv");
    map<int64_t, string> depthByT;
    for (auto &d : depthRows) depthByT[d.t] = d.file;
    cerr << "frames " << cam.size() << ", depth " << depthRows.size() << ", imu " << imu.size() << endl;

    ORB_SLAM3::System slam(argv[1], argv[2], ORB_SLAM3::System::IMU_RGBD, false);
    for (size_t c = 0; c < captures.size(); ++c) {
        if (c > 0) {
            slam.Reset();  // as OrbSlamBridge.finishCapture
            cerr << "=== System::Reset() after capture " << c - 1 << endl;
        }
        int ok = 0, fed = 0;
        int64_t last = -1;
        for (int i = captures[c].first; i < captures[c].second && i < (int)cam.size(); ++i) {
            auto it = depthByT.find(cam[i].t);
            if (it == depthByT.end()) continue;
            cv::Mat d = cv::imread(it->second, cv::IMREAD_UNCHANGED);
            cv::Mat im = black.count(i) ? cv::Mat(480, 640, CV_8UC1, cv::Scalar(16)) : cv::imread(cam[i].file, cv::IMREAD_UNCHANGED);
            vector<ORB_SLAM3::IMU::Point> pts;
            if (last >= 0)
                for (auto &m : imu)
                    if (m.t > last && m.t <= cam[i].t)
                        pts.emplace_back(m.ax, m.ay, m.az, m.wx, m.wy, m.wz, m.t * 1e-9);
            cerr << "cap " << c << " frame " << i << (black.count(i) ? " (black)" : "") << " imu " << pts.size() << flush;
            slam.TrackRGBD(im, d, cam[i].t * 1e-9, pts);
            int s = slam.GetTrackingState();
            cerr << " -> state " << s << endl;
            last = cam[i].t;
            ++fed;
            if (s == 2) ++ok;
        }
        cerr << "=== capture " << c << ": fed " << fed << ", OK " << ok << endl;
    }
    slam.Shutdown();
    cerr << "DONE without crash" << endl;
    return 0;
}
