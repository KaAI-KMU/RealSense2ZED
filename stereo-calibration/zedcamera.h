#pragma once
#include <sl/Camera.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp> // imwrite
#include <vector>
#include <thread>
#include <chrono>

class ZedCamera
{
public:
    ZedCamera(const std::string& directory);
    ~ZedCamera();

    cv::Mat getIntrinsics() const;
    std::vector<float> getCoeffs() const;
    void captureAndSave(const int& img_num);
    sl::float3 convert2Dto3D(int i);

private:
    sl::Camera mZed;
    sl::InitParameters mInitParams;
    sl::ERROR_CODE err;
    std::string mDirectory;
    sl::CameraParameters mIntrinsics;
    sl::RuntimeParameters runtime_params;
};

