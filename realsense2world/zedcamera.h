#pragma once
#include <sl/Camera.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp> // imwrite

class ZedCamera
{
public:
    ZedCamera(const std::string& directory);
    ~ZedCamera();

    sl::CameraParameters getIntrinsicParam();
    void captureAndSave();

private:
    sl::Camera mZed;
    sl::InitParameters mInitParams;
    sl::ERROR_CODE err;
    sl::CalibrationParameters mLeftCalibrationParams;
    std::string mDirectory;
};

