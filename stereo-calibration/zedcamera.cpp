#include "zedcamera.h"

ZedCamera::ZedCamera(const std::string& directory)
    : mDirectory(directory)
{
    // Set configuration parameters
    mInitParams.camera_resolution = sl::RESOLUTION::HD1080;
    mInitParams.camera_fps = 30;

    // Open the ZED camera
    err = mZed.open(mInitParams);
    if (err != sl::ERROR_CODE::SUCCESS) {
        std::cout << "Failed to open ZED camera: " << sl::toString(err) << std::endl;
        exit(1);
    }
}

ZedCamera::~ZedCamera()
{
    // Close the camera
    mZed.close();
    exit(EXIT_SUCCESS);
}

sl::CameraParameters ZedCamera::getIntrinsicParam()
{
    // Calibration
    mLeftCalibrationParams = mZed.getCameraInformation().camera_configuration.calibration_parameters;
    // Return Left camera's parameters
    return mLeftCalibrationParams.left_cam;
}

void ZedCamera::captureAndSave()
{
    sl::Mat zed_image;
    err = mZed.grab();
    // A new image is available if grab() returns ERROR_CODE::SUCCESS
    if (err == sl::ERROR_CODE::SUCCESS) {

        // Get the left image
        mZed.retrieveImage(zed_image, sl::VIEW::LEFT);

        // Display the image resolution and its acquisition timestamp
        std::cout<<"Image resolution: "<< zed_image.getWidth()<<"x"
                  <<zed_image.getHeight() <<" || Image timestamp: "
                  <<zed_image.timestamp.data_ns<<std::endl;

        // Save image in build/images/
        zed_image.write((mDirectory + "left-zed.png").c_str());

    }
}
