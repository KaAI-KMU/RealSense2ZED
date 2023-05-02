#pragma once
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp> // imwrite

class RealSenseCamera
{
public:
    RealSenseCamera(const std::string& directory);
    ~RealSenseCamera();

    rs2_intrinsics getIntrinsicParam();
    void captureAndSave() const;

private:
    rs2::pipeline mPipe;
    rs2::stream_profile mDepthStreamProfile;
    rs2_intrinsics cDepthIntrinsicsParam;
    std::string mDirectory;
};

