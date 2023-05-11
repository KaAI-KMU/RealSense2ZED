#include "zedcamera.h"

ZedCamera::ZedCamera(const std::string& directory)
    : mDirectory(directory)
{
    // Set configuration parameters
    mInitParams.coordinate_system = sl::COORDINATE_SYSTEM::IMAGE; // Right-haned
    mInitParams.camera_resolution = sl::RESOLUTION::AUTO; // Resolution
    mInitParams.camera_fps = 30; // Frame
    mInitParams.depth_mode = sl::DEPTH_MODE::ULTRA;
    mInitParams.coordinate_units = sl::UNIT::MILLIMETER; // Use millimeter units (for depth measurements)

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

        // Save image in build/images/
        zed_image.write(fileName.c_str());
    }
}

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/calib3d/calib3d_c.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv_modules.hpp>

sl::float3 ZedCamera::convert2Dto3D(int i)
{
    sl::Mat image, depth, point_cloud;
    sl::float4 point_cloud_value;

    runtime_params.measure3D_reference_frame = sl::REFERENCE_FRAME::WORLD;
    mZed.grab(runtime_params);

    if (err == sl::ERROR_CODE::SUCCESS) {
        mZed.retrieveImage(image, sl::VIEW::LEFT);
        mZed.retrieveMeasure(depth, sl::MEASURE::DEPTH);
        mZed.retrieveMeasure(point_cloud, sl::MEASURE::XYZRGBA);
        image.write((mDirectory + "image" + std::to_string(i) + ".png").c_str());
        depth.write((mDirectory + "depth" + std::to_string(i) + ".png").c_str());

        int x = image.getWidth() / 2;
        int y = image.getHeight() / 2;

        point_cloud.getValue(x, y, &point_cloud_value);
        if(std::isfinite(point_cloud_value.z)){
            float distance = sqrt(point_cloud_value.x * point_cloud_value.x + point_cloud_value.y * point_cloud_value.y + point_cloud_value.z * point_cloud_value.z);
            std::cout<<"Distance to Camera at {"<<x<<";"<<y<<"}: "<<distance<<"mm"<<std::endl;
        }else
            std::cout<<"The Distance can not be computed at {"<<x<<";"<<y<<"}"<<std::endl;
        std::cout << "point_cloud: " << point_cloud_value << std::endl;
    }

    return point_cloud_value;
}
