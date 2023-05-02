#include "realsensecamera.h"

RealSenseCamera::RealSenseCamera(const std::string& directory)
    : mDirectory(directory)
{
    mPipe.start();
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
    rs2::frameset frames = mPipe.wait_for_frames();
    rs2::frame color_frame = frames.get_color_frame();
    cv::Mat color_mat(cv::Size(640, 480), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
    cv::imwrite(mDirectory + "/realsense.png", color_mat);
}
