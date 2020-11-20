// CamRec.cpp : Defines the entry point for the application.

#include "CamRec.h"

#include <iostream>

using namespace std;

std::string exec(const char* cmd) {
    std::array<char, 128> buffer;
    std::string result;
    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);
    if (!pipe) {
        throw std::runtime_error("popen() failed!");
    }
    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
        result += buffer.data();
    }
    return result;
}

int main()
{
    cv::VideoCapture left(0);
    cv::VideoCapture right(1);

    if (!left.isOpened() || !right.isOpened()) {
        std::cerr << "ERROR: Could not open camera" << std::endl;
        return 1;
    }

    cv::Mat imageL, imageR;

    left >> imageL;
    right >> imageR;

    // Fisheye correction: https://medium.com/@kennethjiang/calibrate-fisheye-lens-using-opencv-333b05afa0b0

    // K and D from correction.py
    cv::Mat K(3, 3, cv::DataType<float>::type);
    K.at<float>(0, 0) = 528.53618582196384f;
    K.at<float>(0, 1) = 0.0f;
    K.at<float>(0, 2) = 314.01736116032430f;

    K.at<float>(1, 0) = 0.0f;
    K.at<float>(1, 1) = 532.01912214324500f;
    K.at<float>(1, 2) = 231.43930864205211f;

    K.at<float>(2, 0) = 0.0f;
    K.at<float>(2, 1) = 0.0f;
    K.at<float>(2, 2) = 1.0f;

    cv::Mat D(5, 1, cv::DataType<float>::type);
    D.at<float>(0, 0) = -0.11839989180635836f;
    D.at<float>(1, 0) = 0.25425420873955445f;
    D.at<float>(2, 0) = 0.0013269901775205413f;
    D.at<float>(3, 0) = 0.0015787467748277866f;
    D.at<float>(4, 0) = -0.11567938093172066f;

    // Calculate mapping matrices
    cv::Mat map1, map2;
    cv::initUndistortRectifyMap(K, D, cv::Mat(), K, cv::Size(imageL.cols, imageL.rows), CV_32FC1, map1, map2);

    left >> imageL;
    right >> imageR;

    // Undistort
    cv::Mat undistL, undistR;
    cv::remap(imageL, undistL, map1, map2, cv::INTER_LINEAR);
    cv::remap(imageR, undistR, map1, map2, cv::INTER_LINEAR);

    // Further iamge manipulation (resize, crop, rotate)

    // Save images
    cv::imwrite("/tmp/left.png", undistL);
    cv::imwrite("/tmp/left.png", undistR);


    // OCR (--psm 10 == one character)
    string leftChar = exec("tesseract /tmp/left.png stdout --psm 10");
    string rightChar = exec("tesseract /tmp/right.png stdout --psm 10");

	getchar();
	return 0;
}
