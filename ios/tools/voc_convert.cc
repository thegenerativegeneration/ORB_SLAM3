// Converts the ORB-SLAM3 text vocabulary to the DBoW2 binary format and checks the round trip.
// Usage: voc_convert ORBvoc.txt ORBvoc.bin
#include <chrono>
#include <iostream>
#include "ORBVocabulary.h"

using Clock = std::chrono::steady_clock;
static double Ms(Clock::time_point a, Clock::time_point b) { return std::chrono::duration<double, std::milli>(b - a).count(); }

int main(int argc, char** argv)
{
    if (argc != 3) { std::cerr << "usage: voc_convert <in.txt> <out.bin>\n"; return 1; }
    ORB_SLAM3::ORBVocabulary txt, bin;
    auto t0 = Clock::now();
    if (!txt.loadFromTextFile(argv[1])) { std::cerr << "text load failed\n"; return 1; }
    auto t1 = Clock::now();
    if (!txt.saveToBinaryFile(argv[2])) { std::cerr << "binary save failed\n"; return 1; }
    auto t2 = Clock::now();
    if (!bin.loadFromBinaryFile(argv[2])) { std::cerr << "binary load failed\n"; return 1; }
    auto t3 = Clock::now();
    std::cout << "text load " << Ms(t0, t1) << " ms, binary save " << Ms(t1, t2) << " ms, binary load " << Ms(t2, t3) << " ms\n";
    std::cout << "words " << txt.size() << " / " << bin.size() << ", k " << txt.getBranchingFactor() << ", L " << txt.getDepthLevels() << "\n";

    // Round trip: same words and weights, same descriptors.
    if (txt.size() != bin.size()) { std::cerr << "MISMATCH size\n"; return 2; }
    for (unsigned int w = 0; w < txt.size(); ++w) {
        if (txt.getWordWeight(w) != bin.getWordWeight(w) || cv::norm(txt.getWord(w), bin.getWord(w), cv::NORM_HAMMING) != 0) {
            std::cerr << "MISMATCH word " << w << "\n";
            return 2;
        }
    }
    // Tree structure: random descriptors must map to identical BoW/feature vectors.
    cv::RNG rng(42);
    std::vector<cv::Mat> desc;
    for (int i = 0; i < 5000; ++i) { cv::Mat d(1, 32, CV_8U); rng.fill(d, cv::RNG::UNIFORM, 0, 256); desc.push_back(d); }
    DBoW2::BowVector bv1, bv2;
    DBoW2::FeatureVector fv1, fv2;
    txt.transform(desc, bv1, fv1, 4);
    bin.transform(desc, bv2, fv2, 4);
    if (bv1 != bv2 || fv1 != fv2) { std::cerr << "MISMATCH transform\n"; return 2; }
    std::cout << "round trip OK (word descriptors/weights and transform of 5000 random descriptors identical)\n";
    return 0;
}
