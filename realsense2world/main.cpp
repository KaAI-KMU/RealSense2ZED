#include <iostream>
#include "realsensecamera.h"
#include "zedcamera.h"
#include <filesystem>

using namespace std;

int main(int argc, char **argv)
{
    // 현재 실행 파일의 경로를 얻어옴
    string exe_path = std::filesystem::path(argv[0]).parent_path();
    string directory = (exe_path + "/images/");
    filesystem::create_directories(directory);

    // Get Realsense intrinsic parameter
    RealSenseCamera realsense(directory);
    auto const depth_real = realsense.getIntrinsicParam();
    realsense.captureAndSave();

    // Get ZED intrinsic parameter
    ZedCamera zed(directory);
    auto const left_zed = zed.getIntrinsicParam();
    zed.captureAndSave();

    return 0;
}
