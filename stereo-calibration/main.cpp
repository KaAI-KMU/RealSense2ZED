#include <iostream>
#include <filesystem>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/calib3d/calib3d_c.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv_modules.hpp>

#include "realsensecamera.h"
#include "zedcamera.h"

using namespace std;

int main(int argc, char **argv)
{
    // 현재 실행 파일의 경로를 얻어옴
    string exe_path = std::filesystem::path(argv[0]).parent_path();
    string directory_depth = (exe_path + "/images/depth/");
//    string directory_zed = (exe_path + "/images/zed/");
//    string directory_rs = (exe_path + "/images/realsense/");

    // 각각의 카메라 사진들을 저장할 디렉토리 생성
    filesystem::create_directories(directory_depth);
//    filesystem::create_directories(directory_zed);
//    filesystem::create_directories(directory_rs);

//    // Realsense 카메라 내부 파라미터, 왜곡 파라미터
//    RealSenseCamera realsense(directory_rs);
//    auto K_rs = realsense.getIntrinsics();
//    auto D_rs = realsense.getCoeffs();

    // ZED 카메라 내부 파라미터, 왜곡 파라미터
    ZedCamera zed(directory_depth);
    auto K_zed = zed.getIntrinsics();
    auto D_zed = zed.getCoeffs();

    /// 만약 사진을 찍어야 한다면 다음 코드 이용
    /// 나머지 코드는 주석처리

//    for (auto i = 0; i < 30; i++) {
//        realsense.captureAndSave(i);
//        zed.captureAndSave(i);
//        std::this_thread::sleep_for(std::chrono::milliseconds(500)); // 0.5초 딜레이
//    }

    /// 2D image point to 3D point
    for (auto i = 0; i < 30; i++)  {
        sl::float3 depth_point = zed.convert2Dto3D(i);
        std::this_thread::sleep_for(std::chrono::milliseconds(200)); // 0.5초 딜레이
    }

/*
    // 체크보드 사이즈 정의
    const int board_width{6};
    const int board_height{9};
    const cv::Size board_size = cv::Size(board_width, board_height);

    // 이미지 읽어오기
    vector<cv::String> zed_img, rs_img;
    cv::glob(directory_zed, zed_img);
    cv::glob(directory_rs, rs_img);

    cout << "Image size: " << zed_img.size() << endl;
    if (zed_img.size() == 0) {
        cout << "No images\n" << endl;
        return 0;
    }

    // 각 이미지에 대해 3D 좌표를 저장하는 벡터 선언
    vector<vector<cv::Point3f>> object_points;

    // 각 이미지에 대해 2D 좌표를 저장하는 벡터 선언
    vector<vector<cv::Point2f>> imagePoint_zed;
    vector<vector<cv::Point2f>> imagePoint_rs;
    vector<vector<cv::Point2f>> zed_img_points;
    vector<vector<cv::Point2f>> rs_img_points;

    // World 좌표계 3D 포인트 좌표
    vector<cv::Point3f> objp;

    // 월드 좌표계 objp에는 54개의 좌표가 저장되어 있음
    // 실세계 좌표이며 (0,0,0) 다음에 (1, 0, 0)이 아니라 square length를 곱해서 실제 거리로 저장되어있는 좌표
    // 실제 값에 대한 정보가 들어가야 하므로 j i 가 아니라 체스보드의 한칸 길이(나의 경우 2.5cm)까지 곱해서 넣어줘어야 함
    for (int i = 0; i < board_height; i++)		// board_height = 6
        for (int j = 0; j < board_width; j++)	// board_width = 9
            objp.push_back(cv::Point3f(j * 25, i * 25, 0));	// z는 0 이고 (x, y ,z)로 담기니까 (j, i , 0)으로 벡터에 push_back으로 값 담기



    cv::Mat img1, img2, gray1, gray2;
    // 검출된 check board corner 포인트(2D 좌표)를 담을 벡터 선언
    vector<cv::Point2f> corner_pts1, corner_pts2;

    // findChessboardCorners 되었는지 안 되었는지를 확인하기 위한 용도
    bool found_zed, found_rs;
    cv::Size newSize(640, 480);
    cv::Rect roi(0, 0, 1440, 1080);

    // 이미지 개수만큼 for loop
    for (int i = 0; i < zed_img.size(); i++)
    {
        // images로부터 한 장씩 받아서 frame으로 읽어옴
        img1 = cv::imread(zed_img[i]);
        img2 = cv::imread(rs_img[i]);

        // image crop and resize
        img1 = img1(roi);
        cv::resize(img1, img1, newSize);

        // COLOR 니까 GRAY로 바꾸기 위해 cvtColor 함수를 사용해 변경
        cv::cvtColor(img1, gray1, cv::COLOR_BGR2GRAY);
        cv::cvtColor(img2, gray2, cv::COLOR_BGR2GRAY);

        // 체크보드 코너 개수
        // board_size와 같은 개수를 찾으면 true 아니면 false
        found_zed = cv::findChessboardCorners(gray1, board_size, corner_pts1, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
        found_rs = cv::findChessboardCorners(gray2, board_size, corner_pts2, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);

        // 둘 중 하나라도 false면 해당 image 이름 출력
        if (!found_zed) {
            cout << "findChessboardCorners Error!: ";
            cout << "zed image name: " << zed_img[i] << endl;
        }
        if (!found_rs) {
            cout << "findChessboardCorners Error!";
            cout << "realsense image name: " << rs_img[i] << endl;
        }

        // 기준 인스턴스 생성
        cv::TermCriteria criteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1);

        // 주어진 2D point에 대해 더 정제시켜 업데이트
        // Chessboard Corner 그리기
        if (found_zed)
        {
            cv::cornerSubPix(gray1, corner_pts1, cv::Size(11, 11), cv::Size(-1, -1), criteria);
            cv::drawChessboardCorners(gray1, board_size, corner_pts1, found_zed);
//            cv::imshow("gray1", gray1);
        }

        if (found_rs)
        {
            cv::cornerSubPix(gray2, corner_pts2, cv::Size(11, 11), cv::Size(-1, -1), criteria);
            cv::drawChessboardCorners(gray2, board_size, corner_pts2, found_rs);
//            cv::imshow("gray2", gray2);
//            cv::waitKey(0);
        }

        if (found_zed && found_rs) {
            cout << i << ". Found corners!" << endl;
            imagePoint_zed.push_back(corner_pts1); // 해당 번째의 corner_pts1의 값을 imgopoints_zed에 추가
            imagePoint_rs.push_back(corner_pts2);  // 해당 번째의 corner_pts2의 값을 imgopoints_rs에 추가
            object_points.push_back(objp);         // 해당 번째의 objp의 값을 objpoints에 추가
        }
    }

    // Stereo Calibration 시작
    cv::Mat R, T, E, F;

    cv::stereoCalibrate(object_points, zed_img_points, rs_img_points,
                        K_zed, D_zed, K_rs, D_rs, newSize,
                        R, T, E, F);
    // Stereo Calibration Parameters
    cout << "[Stereo Camera parameters]\n";
    cout << "Rotation Matrix\n" << R << "\n\n";
    cout << "Translation Vector\n" << T << "\n\n";
    cout << "Essential Matrix\n" << E << "\n\n";
    cout << "Fundamental Matrix\n" << F << "\n\n\n";

    cv::Mat R1, R2, P1, P2, Q;
    cv::Rect validRoi[2];

    cv::stereoRectify(K_zed, D_zed, K_rs, D_rs, newSize,
                      R, T, R1, R2, P1, P2, Q, cv::CALIB_ZERO_DISPARITY, 1, newSize, &validRoi[0], &validRoi[1]);

    // 양 카메라의 이미지 평면을 같은 평면으로 바꾸는 변환 행렬을 계산
    cout << "[Stereo Rectify parameters]\n";
    cout << "R1\n" << R1 << "\n\n";
    cout << "R2\n" << R2 << "\n\n";
    cout << "P1\n" << P1 << "\n\n";
    cout << "P2\n" << P2 << "\n\n";
    cout << "Q\n" << Q << "\n\n\n";

    // Realsense 3D 좌표 (x, y, z) 초기화
    double x = 10;
    double y = 20;
    double z = 30;

    // Realsense 3D 좌표를 행렬로 변환
    cv::Mat rs_point = (cv::Mat_<double>(3,1) << x, y, z);

    // Realsense 좌표를 ZED 좌표계로 변환
    cv::Mat zed_point = R * rs_point + T;

    // ZED 좌표계의 (x, y, z) 값을 출력
    cout << "ZED 좌표계의 x : " << zed_point.at<double>(0) << endl;
    cout << "ZED 좌표계의 y : " << zed_point.at<double>(1) << endl;
    cout << "ZED 좌표계의 z : " << zed_point.at<double>(2) << endl;
*/
    return 0;
}
