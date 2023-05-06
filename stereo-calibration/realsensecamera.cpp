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
    mDepthStreamProfile = mPipe.get_active_profile().get_stream(RS2_STREAM_DEPTH);
}

RealSenseCamera::~RealSenseCamera()
{
    mPipe.stop();
}

rs2_intrinsics RealSenseCamera::getIntrinsicParam()
{
    cDepthIntrinsicsParam = mDepthStreamProfile.as<rs2::video_stream_profile>().get_intrinsics();
    return cDepthIntrinsicsParam;
}

void RealSenseCamera::captureAndSave() const
{
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

    if (!cv::imwrite(mDirectory + "/right-realsense.png", color_mat)) {
        throw std::runtime_error("Failed to save color image to disk.");
    }
}
