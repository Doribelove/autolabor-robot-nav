#include <opencv2/core.hpp>
#include <opencv2/core/ocl.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

// Validate the installed CPU stereo engine with a known 16-pixel disparity.
// This is deliberately independent of any camera or production ROS graph.
int main() {
    cv::setNumThreads(2);
    cv::ocl::setUseOpenCL(false);
    cv::Mat left(120, 320, CV_8U), right(120, 320, CV_8U, cv::Scalar(0));
    cv::RNG random(23748636); random.fill(left, cv::RNG::UNIFORM, 0, 255);
    left(cv::Rect(16, 0, 304, 120)).copyTo(right(cv::Rect(0, 0, 304, 120)));
    const auto matcher = cv::StereoSGBM::create(0, 64, 5, 8*25, 32*25, 1, 31, 5, 0, 0);
    cv::Mat disparity; matcher->compute(left, right, disparity);
    std::vector<double> values;
    for (int y = 10; y < 110; ++y)
        for (int x = 80; x < 290; ++x) {
            const double d = disparity.at<short>(y, x)/16.0;
            if (d > 0) values.push_back(d);
        }
    if (values.empty()) return 1;
    std::sort(values.begin(), values.end());
    const double median = values[values.size()/2];
    const bool ok = values.size() > 20000 && std::abs(median-16.0) < 0.25;
    std::cout << "{\"opencv_version\":\"" << CV_VERSION
              << "\",\"cpu_stereo_self_test\":" << (ok ? "true" : "false")
              << ",\"expected_disparity_px\":16,\"median_disparity_px\":" << median
              << ",\"valid_pixels\":" << values.size()
              << ",\"camera_data_used\":false,\"real_depth_accuracy_validated\":false}\n";
    return ok ? 0 : 1;
}
