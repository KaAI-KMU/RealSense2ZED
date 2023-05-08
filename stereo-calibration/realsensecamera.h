#pragma once
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp> // imwrite
#include <vector>
#include <chrono>
#include <thread>

class RealSenseCamera
{
public:
    RealSenseCamera(const std::string& directory);
    ~RealSenseCamera();

    cv::Mat getIntrinsics() const;
    std::vector<float> getCoeffs() const;
    void captureAndSave(const int& img_num) const;

private:
    rs2::pipeline mPipe;
    rs2::stream_profile mDepthStreamProfile;
    std::string mDirectory;
    rs2_intrinsics mIntrinsics;
};

