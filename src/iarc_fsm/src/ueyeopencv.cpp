#include <UEyeOpenCV.hpp>
#include <iostream>
#include <ueye.h>

UeyeOpencvCam::UeyeOpencvCam(int wdth, int heigh) {
        width = wdth;
        height = heigh;

        mattie = cv::Mat(height, width, CV_8UC3);

        hCam = 1;

        char* ppcImgMem;
        int pid;
        INT nAOISupported = 0;

        is_InitCamera(&hCam, NULL);
        is_SetColorMode(hCam, IS_CM_BGR8_PACKED);

        INT nGamma = 180;
        is_Gamma(hCam, IS_GAMMA_CMD_SET, (void*) &nGamma, sizeof(nGamma));

        is_ImageFormat(hCam, IMGFRMT_CMD_GET_ARBITRARY_AOI_SUPPORTED, (void*) &nAOISupported, sizeof(nAOISupported));
        is_AllocImageMem(hCam, width, height, 24, &ppcImgMem, &pid);
        is_SetImageMem(hCam, ppcImgMem, pid);
        is_CaptureVideo(hCam, IS_WAIT);
}


cv::Mat UeyeOpencvCam::getFrame() {
    VOID* pMem;
    int retInt = is_GetImageMem(hCam, &pMem);
    if (retInt != IS_SUCCESS) {
        if (retInt == IS_INVALID_CAMERA_HANDLE){
            std::cout<<"error CAMERA_HANDLE"<<std::endl;
        }
        else{
                std::cout<<"error is_GetImageMem"<<std::endl;
        }
    }
    else{
            memcpy(mattie.ptr(), pMem, width * height * 3);
    }
    return mattie;
}

