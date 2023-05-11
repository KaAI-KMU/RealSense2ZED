#include "realsensecamera.h"

RealSenseCamera::RealSenseCamera(const std::string& directory)
    : mDirectory(directory)
{
    rs2::context ctx;
    auto devices = ctx.query_devices();
    if (devices.size() == 0) {
        throw std::runtime_error("No RealSense devices found.");
    }
    rs2::device device = devices.front();

    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);

    // Start the pipeline
    mPipe.start(cfg);
    mDepthStreamProfile = mPipe.get_active_profile().get_stream(RS2_STREAM_COLOR);

    // 내부 파라미터(K), 왜곡 파라미터(D)를 포함하는 struct 생성
    mIntrinsics = mDepthStreamProfile.as<rs2::video_stream_profile>().get_intrinsics();
}

RealSenseCamera::~RealSenseCamera()
{
    mPipe.stop();
}

cv::Mat RealSenseCamera::getIntrinsics() const
{
    cv::Mat K = cv::Mat::eye(3, 3, CV_64F); // create a 3x3 identity matrix
    K.at<double>(0, 0) = mIntrinsics.fx;
    K.at<double>(1, 1) = mIntrinsics.fy;
    K.at<double>(0, 2) = mIntrinsics.ppx;
    K.at<double>(1, 2) = mIntrinsics.ppy;
    K.at<double>(2, 2) = 1.0;

    return K;
}

std::vector<float> RealSenseCamera::getCoeffs() const
{
    std::vector<float> D;
    for (auto i = 0; i < 5; i++)
        D.push_back(mIntrinsics.coeffs[i]);

    //    float k1 = mIntrinsics.coeffs[0];
    //    float k2 = mIntrinsics.coeffs[1];
    //    float k3 = mIntrinsics.coeffs[2];
    //    float p1 = mIntrinsics.coeffs[3];
    //    float p2 = mIntrinsics.coeffs[4];

    return D;
}

void RealSenseCamera::captureAndSave(const int& img_num) const
{
    std::string fileName = mDirectory + std::to_string(img_num) + ".png";
    rs2::frameset frames = mPipe.wait_for_frames(5000);
    if (frames.size() < 2) {
        throw std::runtime_error("Failed to capture frames from the camera.");
    }

    rs2::frame color_frame = frames.get_color_frame();
    if (!color_frame) {
        throw std::runtime_error("Failed to get color frame from the camera.");
    }

    cv::Mat color_mat(cv::Size(640, 480), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
    if (color_mat.empty()) {
        throw std::runtime_error("Failed to convert color frame to OpenCV matrix.");
    }

    if (!cv::imwrite(fileName, color_mat)) {
        throw std::runtime_error("Failed to save color image to disk.");
    }
}
