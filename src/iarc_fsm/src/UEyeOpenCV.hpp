#ifndef UEYEOPENCV_HPP
#define UEYEOPENCV_HPP

#pragma once
#include <ueye.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

class UeyeOpencvCam {
public:
        UeyeOpencvCam(int wdth, int heigh);
        cv::Mat getFrame();

private:
        HIDS hCam;
        cv::Mat mattie;
        int width;
        int height;
};

#endif // UEYEOPENCV_HPP
