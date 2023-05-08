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
    mIntrinsics = mZed.getCameraInformation().camera_configuration.calibration_parameters.left_cam;
}

ZedCamera::~ZedCamera()
{
    // Close the camera
    mZed.close();
    exit(EXIT_SUCCESS);
}

cv::Mat ZedCamera::getIntrinsics() const
{
    cv::Mat cv_intrinsics = cv::Mat::eye(3, 3, CV_64F); // create a 3x3 identity matrix
    cv_intrinsics.at<double>(0, 0) = mIntrinsics.fx;
    cv_intrinsics.at<double>(1, 1) = mIntrinsics.fy;
    cv_intrinsics.at<double>(0, 2) = mIntrinsics.cx;
    cv_intrinsics.at<double>(1, 2) = mIntrinsics.cy;
    cv_intrinsics.at<double>(2, 2) = 1.0;

    return cv_intrinsics;
}

std::vector<float> ZedCamera::getCoeffs() const
{
    std::vector<float> D;
    for (auto i = 0; i < 5; i++)
        D.push_back(mIntrinsics.disto[i]);

    //    float k1 = mIntrinsics.disto[0];
    //    float k2 = mIntrinsics.disto[1];
    //    float k3 = mIntrinsics.disto[2];
    //    float p1 = mIntrinsics.disto[3];
    //    float p2 = mIntrinsics.disto[4];

    return D;
}

void ZedCamera::captureAndSave(const int& img_num)
{
    std::string fileName = mDirectory + std::to_string(img_num) + ".png";
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
        zed_image.write(fileName.c_str());
    }
    // 0.3초 딜레이
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
}
