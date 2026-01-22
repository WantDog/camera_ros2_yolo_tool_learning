#ifndef RADAR_STATION_CAMERADRIVE_H
#define RADAR_STATION_CAMERADRIVE_H

#include <opencv2/opencv.hpp>
#include "MvCameraControl.h"
#include <chrono>
#include <fstream>
#include <sstream>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <iostream>


struct BaseInitStruct {
    int width;
    int height;
    int fps;
    int video_capture_api;
};

struct USBInitSturct : BaseInitStruct {
    int index;
    std::string fourcc;
};

struct HIKInitStruct : BaseInitStruct {
    int camera_mode;
    int exposure_time;
};

using CallbackType = void (*)(unsigned char*, MV_FRAME_OUT_INFO_EX*, void*);

void camCallBack(unsigned char *pData, MV_FRAME_OUT_INFO_EX *pFrameInfo, void *pUser);

class HK_Camera {

public:
    HK_Camera();
    HK_Camera(CallbackType callback, void* node_this); // 在使用类的成员函数时需要使用 第二个参数，传入 this 指针 (main.cpp中有示例)
    ~HK_Camera();
    unsigned int findConnectableUSBDevice(); // USB MV-CS016-10UC 相机
    bool cameraInit(const HIKInitStruct &t); // 适配情况同上， 根据自己需求改改
    void startCamera();
    void stopCamera();
    void getFrame(cv::Mat &img);
private:
    MV_CC_DEVICE_INFO_LIST device_list_;
    MV_CC_DEVICE_INFO* device_info_;
    CallbackType call_back_ptr_;
    void* handle_; // 相机句柄
    void* hik_camera_node_;
    int nRet_; // 状态码（通用）
    int camera_mode_; // 0 回调函数 1 主动取帧
};


// 相机类型 USB : 0 \ hik : 1
#define USB 0
#define HIK 1

class Camera {
public:

    Camera(const std::string initFilePath);

    ~Camera();

    bool getFrame(cv::Mat &img);
    bool isOpened();
    int getFPS();
    void open();
    void stop();

    double cameraCalibrate(const std::string& imgPath, int board_width = 9, int board_height = 7, float square_size = 2.5f);
    cv::Vec3f coordinate_;
    cv::Vec3f euler_angles_;
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_matrix_;
private:
    HK_Camera *hik_cam_;
    int camera_type_;
    cv::VideoCapture *cap_;
    std::string init_file_path_;
    bool camera_is_open_;
    HIKInitStruct hik_init_;
    USBInitSturct usb_init_;
};

#endif //RADAR_STATION_CAMERADRIVE_H
