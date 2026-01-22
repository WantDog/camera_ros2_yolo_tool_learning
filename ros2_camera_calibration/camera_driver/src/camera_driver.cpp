#include "camera_driver.h"

void camCallBack(unsigned char *pData, MV_FRAME_OUT_INFO_EX *pFrameInfo, void *pUser) {

    static int count = 0;
    static auto start = std::chrono::high_resolution_clock::now();

    count++;
    auto now = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = now - start;

    if (elapsed.count() >= 1.0) { // 每秒输出一次
        std::cout << "fps : " << count << std::endl;
        count = 0;
        start = now;
    }

    cv::Mat img = cv::Mat(pFrameInfo->nHeight, pFrameInfo->nWidth, CV_8UC3, pData);
    cv::cvtColor(img, img, cv::COLOR_RGB2BGR);
    cv::imshow("cap_img", img);
    cv::waitKey(1);

}

HK_Camera::HK_Camera() {
    call_back_ptr_ = nullptr;
    hik_camera_node_ = nullptr;
    handle_ = nullptr;
    device_info_ = nullptr;
    camera_mode_ = 1;
}

HK_Camera::HK_Camera(CallbackType callback, void* node_this = nullptr) {
    call_back_ptr_ = callback;
    hik_camera_node_ = node_this;
    handle_ = nullptr;
    device_info_ = nullptr;
    camera_mode_ = 0;
}

HK_Camera::~HK_Camera() {
    nRet_ =  MV_CC_StopGrabbing(handle_);
    if(nRet_ != MV_OK){
        printf("MV_CC_StopGrabbing fail! nRet [%x]\n", nRet_);
    } 
    nRet_ =  MV_CC_CloseDevice(handle_);
    if(nRet_ != MV_OK){
        printf("MV_CC_CloseDevice fail! nRet [%x]\n", nRet_);
    } 
    // 销毁句柄
    // destroy handle
    nRet_ = MV_CC_DestroyHandle(handle_);
    if (MV_OK != nRet_)
    {
        printf("MV_CC_DestroyHandle fail! nRet [%x]\n", nRet_);
    }
    handle_ = NULL;

    // ch:反初始化SDK | en:Finalize SDK
    MV_CC_Finalize();
    printf("exit.\n");

}

void HK_Camera::stopCamera() {
    MV_CC_StopGrabbing(handle_);
    MV_CC_CloseDevice(handle_);
}

unsigned int HK_Camera::findConnectableUSBDevice() {
    int nRet = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list_);
    if (device_list_.nDeviceNum > 0) {
        std::cout << "发现 " << device_list_.nDeviceNum << " 个设备 : " << std::endl;
        for (unsigned int i = 0; i < device_list_.nDeviceNum; i++) {
            device_info_ = device_list_.pDeviceInfo[i];
            if (device_info_ == nullptr) break;
            std::cout << "--------------------------------------------" << "\n第 " << i + 1 << " 个 : " << std::endl;
            if (device_info_->nTLayerType == MV_USB_DEVICE && MV_CC_IsDeviceAccessible(device_info_, MV_ACCESS_Monitor)) {
                std::cout << "idProduct : " << device_info_->SpecialInfo.stUsb3VInfo.idProduct << std::endl;
                std::cout << "idVendor : " << device_info_->SpecialInfo.stUsb3VInfo.idVendor << std::endl;
                std::cout << "chDeviceGUID : " << device_info_->SpecialInfo.stUsb3VInfo.chDeviceGUID << std::endl;
                std::cout << "chVendorName : " << device_info_->SpecialInfo.stUsb3VInfo.chVendorName << std::endl;
                std::cout << "chModelName : " << device_info_->SpecialInfo.stUsb3VInfo.chModelName << std::endl;
                std::cout << "chDeviceVersion : " << device_info_->SpecialInfo.stUsb3VInfo.chDeviceVersion << std::endl;
                std::cout << "chSerialNumber : " << device_info_->SpecialInfo.stUsb3VInfo.chSerialNumber << std::endl;
                std::cout << "nDeviceAddress : " << device_info_->SpecialInfo.stUsb3VInfo.nDeviceAddress << std::endl;
            }
            std::cout << "--------------------------------------------" << std::endl;
        }
        if (device_list_.nDeviceNum > 1) {
            int index = -1;
            while (index < 1 && index > device_list_.nDeviceNum) {
                std::cout << "请输入您要连接的设备索引号 : ( 1 到 " << device_list_.nDeviceNum << " )" << std::endl;
                std::cin >> index;
            }
            device_info_ = device_list_.pDeviceInfo[index - 1];
        } else {
            device_info_ = device_list_.pDeviceInfo[0];
        }
    } else {
        std::cout << "未找到可用设备" << std::endl;
    }
    return device_list_.nDeviceNum;
}

bool HK_Camera::cameraInit(const HIKInitStruct &t) {

    nRet_ = MV_CC_Initialize();
    if (MV_OK != nRet_)
    {
        printf("Initialize SDK fail! nRet [0x%x]\n", nRet_);
        return false;
    }    

    int test_num = 3;
    while (device_info_ == nullptr && test_num--) {
        findConnectableUSBDevice();
        sleep(1);
    }
    if (device_info_ == nullptr && test_num == 0) return false;

    nRet_ = MV_CC_CreateHandle(&handle_, device_info_);
    if (MV_OK != nRet_) {
        printf("[CameraInit] : Can not create camera handle! nRet_ [0x%x]\n", nRet_);
        return false;
    }

    nRet_ = MV_CC_OpenDevice(handle_);
    if (MV_OK != nRet_) {
        printf("[CameraInit] : Can not open camera! nRet_ [0x%x]\n", nRet_);
        return false;
    }

    // 获取相机能力 - 查询最大分辨率
    MVCC_INTVALUE_EX stWidthMax = {0};
    MVCC_INTVALUE_EX stHeightMax = {0};
    
    nRet_ = MV_CC_GetIntValueEx(handle_, "WidthMax", &stWidthMax);
    if (MV_OK != nRet_) {
        printf("[CameraInit] : Get WidthMax fail! nRet_ [0x%x]\n", nRet_);
    }
    
    nRet_ = MV_CC_GetIntValueEx(handle_, "HeightMax", &stHeightMax);
    if (MV_OK != nRet_) {
        printf("[CameraInit] : Get HeightMax fail! nRet_ [0x%x]\n", nRet_);
    }
    
    printf("[CameraInit] : Camera max resolution: %d x %d\n", stWidthMax.nCurValue, stHeightMax.nCurValue);
    
    // 设置图像宽高，确保不超过最大值
    int requestedWidth = t.width; // 注意：修改了参数名
    int requestedHeight = t.height;
    
    if (requestedWidth <= 0 || requestedWidth > stWidthMax.nCurValue) {
        requestedWidth = stWidthMax.nCurValue;
    }
    
    if (requestedHeight <= 0 || requestedHeight > stHeightMax.nCurValue) {
        requestedHeight = stHeightMax.nCurValue;
    }

    // 设置图像像素
    nRet_ = MV_CC_SetIntValue(handle_, "Width", requestedWidth);
    printf("[CameraInit] : Setting Width to %d\n", requestedWidth);
    if (MV_OK != nRet_) {
        printf("[CameraInit] : Set Img Width fail [%x]\n", nRet_);
    }
    
    nRet_ = MV_CC_SetIntValue(handle_, "Height", requestedHeight);
    printf("[CameraInit] : Setting Height to %d\n", requestedHeight);
    if (MV_OK != nRet_) {
        printf("[CameraInit] : Set Img Height fail [%x]\n", nRet_);
    }

    MVCC_INTVALUE_EX nFPSMaxValue = {t.fps};
    //帧率控制使能，true表示打开，false标识关闭
    nRet_ = MV_CC_SetBoolValue(handle_, "AcquisitionFrameRateEnable", true);
    if (nRet_ != MV_OK) {
        printf("[CameraInit] : Set AcquisitionBurstFrameCountfail nRet_ [0x%x]!\n", nRet_);
    }
    //设置相机帧率，需注意不要超过相机支持的最大的帧率（相机规格书），超过了也没有意义（需要注意的是不同的像素类型支持的帧率也不同）
    nRet_ = MV_CC_SetFloatValue(handle_, "AcquisitionFrameRate", t.fps);
    if (nRet_ != MV_OK) {
        printf("[CameraInit] : Set AcquisitionBurstFrameCountfail nRet_ [0x%x]!\n", nRet_);
    }


    if (t.exposure_time > 0) {
        //设置手动曝光，设置曝光时间
        nRet_ = MV_CC_SetEnumValue(handle_, "ExposureMode", 0);
        nRet_ = MV_CC_SetFloatValue(handle_, "ExposureTime", t.exposure_time);
        if (MV_OK != nRet_) {
            printf("[CameraInit] : Set ExposureTime fail nRet_ [0x%x]!\n", nRet_);
        }
    }
    //设置自动曝光
    nRet_ = MV_CC_SetEnumValue(handle_, "ExposureAuto", 2); // 0：off 1：once 2：Continuous t.exposure_time > 0 ? 0 : 2
    if (MV_OK != nRet_) {
        printf("[CameraInit] : Set ExposureAuto fail nRet_ [0x%x]!\n", nRet_);
    }


    // // 模拟增益设置  -- 增益 增加低光细节，引入噪声 0-12db 
    nRet_ = MV_CC_SetFloatValue(handle_, "Gain", 16.9807);
    if (MV_OK != nRet_) {
        printf("[CameraInit] : Set Gain fail nRet_ [0x%x]\n", nRet_);
    }
    //设置自动增益
    // nRet_ = MV_CC_SetEnumValue(handle_, "GainAuto", 1);
    // if (MV_OK != nRet_) {
    //     printf("[CameraInit] : Set GainAuto fail nRet_ [0x%x]!\n", nRet_);
    // }


    // //1.打开数字增益使能  1 表示线性（没有非线性变换），小于1 的值会使图像更暗 大于1 的值会使图像更亮。 
    // nRet_ = MV_CC_SetBoolValue(handle_, "GammaEnable", true);
    // if (MV_OK != nRet_) {
    //     printf("[CameraInit] : Set GammaEnable fail! nRet_ [0x%x]\n", nRet_);
    // }
    // // //2.设置gamma类型，user：1，sRGB：2  1 -- 手动设置 2 --  自定义设置
    // nRet_ = MV_CC_SetEnumValue(handle_, "GammaSelector", 1);
    // if (MV_OK != nRet_) {
    //     printf("[CameraInit] : Set GammaSelector fail! nRet_ [0x%x]\n", nRet_);
    // }
    // // //3.设置gamma值，推荐范围0.5-2，1为线性拉伸   < 1 : 暗 多细节对比  >1 : 亮 少细节对比
    // nRet_ = MV_CC_SetFloatValue(handle_, "Gamma", 1.8);
    // if (MV_OK != nRet_) {
    //     printf("[CameraInit] : Set Gamma failed! nRet_ [0x%x]\n", nRet_);
    // }

    //开启自动白平衡  消除s色偏
    nRet_ = MV_CC_SetEnumValue(handle_, "BalanceWhiteAuto", true);
    if (MV_OK != nRet_) {
        printf("[CameraInit] : Set BalanceWhiteAuto fail! nRet_ [0x%x]\n", nRet_);
    }

    // 关闭触发模式
    nRet_ = MV_CC_SetEnumValue(handle_, "TriggerMode", false);
    if (MV_OK != nRet_) {
        printf("[CameraInit] : Set TriggerMode fail! nRet_ [0x%x]\n", nRet_);
    }

    // // 3. 设置水平 Binning 因子（例如设置为 2）
    // nRet_ = MV_CC_SetFloatValue(handle_, "BinningHorizontal", 2);
    // if (MV_OK != nRet_) {
    //     std::cerr << "Set BinningHorizontal failed! Error code: " << nRet_ << std::endl;
    //     return -1;
    // }

    // // 4. 设置垂直 Binning 因子（例如设置为 2）
    // nRet_ = MV_CC_SetFloatValue(handle_, "BinningVertical", 2);
    // if (MV_OK != nRet_) {
    //     std::cerr << "Set BinningVertical failed! Error code: " << nRet_ << std::endl;
    //     return -1;
    // }
    // After opening the device, query supported pixel formats
    MVCC_ENUMVALUE pixelFormatInfo = {0};
    nRet_ = MV_CC_GetEnumValue(handle_, "PixelFormat", &pixelFormatInfo);
    if (MV_OK == nRet_) {
        std::cout << "[CameraInit] : Current PixelFormat: " << pixelFormatInfo.nCurValue << std::endl;
        std::cout << "[CameraInit] : Supported PixelFormats: ";
        for (unsigned int i = 0; i < pixelFormatInfo.nSupportedNum; i++) {
            std::cout << pixelFormatInfo.nSupportValue[i] << " ";
        }
        std::cout << std::endl;
        
        // Check if the requested format is supported
        bool formatSupported = false;
        for (unsigned int i = 0; i < pixelFormatInfo.nSupportedNum; i++) {
            if (pixelFormatInfo.nSupportValue[i] == t.video_capture_api) {
                formatSupported = true;
                break;
            }
        }
        
        if (!formatSupported) {
            std::cerr << "[CameraInit] : Requested pixel format " << t.video_capture_api 
                      << " is not supported. Using current format: " << pixelFormatInfo.nCurValue << std::endl;
            
            // You could set a default format here
            // For better color reproduction for MV-CS016-10UC, prefer PixelType_Gvsp_RGB8_Packed if available
            for (unsigned int i = 0; i < pixelFormatInfo.nSupportedNum; i++) {
                if (pixelFormatInfo.nSupportValue[i] == PixelType_Gvsp_RGB8_Packed) {
                    nRet_ = MV_CC_SetEnumValue(handle_, "PixelFormat", PixelType_Gvsp_RGB8_Packed);
                    std::cout << "[CameraInit] : Setting PixelFormat to RGB8_Packed" << std::endl;
                    break;
                }
            }
        } else {
            // Set the requested pixel format
    nRet_ = MV_CC_SetEnumValue(handle_, "PixelFormat", t.video_capture_api);
            std::cout << "[CameraInit] : Setting PixelFormat to " << t.video_capture_api << std::endl;
    if (MV_OK != nRet_) {
                std::cerr << "[CameraInit] : Set PixelFormat fail! nRet_ [0x" << std::hex << nRet_ << "]" << std::endl;
            }
        }
    } else {
        std::cerr << "[CameraInit] : Failed to get pixel format info! nRet_ [0x" << std::hex << nRet_ << "]" << std::endl;
    }

    if (camera_mode_ == 0) {
        MV_CC_RegisterImageCallBackForRGB(handle_, call_back_ptr_, hik_camera_node_);
    }
    return true;
}

void HK_Camera::startCamera() {

    if (device_info_ == nullptr) return;
    nRet_ = MV_CC_StartGrabbing(handle_);
    if( MV_OK != nRet_){
        std::cout  << "failed in start grab" << std::endl;
    }

}
void HK_Camera::getFrame(cv::Mat &img) {
    if (camera_mode_ == 0) {
        std::cerr << "[CameraGetFrame] : 已设置回调函数模式，无法通过此方法获得数据" << std::endl;
        return;
    }

    MV_FRAME_OUT stImageInfo = {0};
    nRet_ = MV_CC_GetImageBuffer(handle_, &stImageInfo, 1000);
    if (nRet_ == MV_OK) {
        if (stImageInfo.pBufAddr == nullptr) {
            std::cerr << "[CameraGetFrame] : Image buffer is null!" << std::endl;
            return;
        }

        // Debug information
        // std::cout << "[CameraGetFrame] : Frame Size: " << stImageInfo.stFrameInfo.nWidth << "x" 
        //           << stImageInfo.stFrameInfo.nHeight << " PixelType: " << stImageInfo.stFrameInfo.enPixelType << std::endl;

        // Handle different pixel formats
        switch (stImageInfo.stFrameInfo.enPixelType) {
            // Mono formats
            case PixelType_Gvsp_Mono8:
                img = cv::Mat(stImageInfo.stFrameInfo.nHeight, stImageInfo.stFrameInfo.nWidth, CV_8UC1);
                memcpy(img.data, stImageInfo.pBufAddr, stImageInfo.stFrameInfo.nHeight * stImageInfo.stFrameInfo.nWidth);
                break;
                
            case PixelType_Gvsp_Mono10:
            case PixelType_Gvsp_Mono12:
            case PixelType_Gvsp_Mono14:
            case PixelType_Gvsp_Mono16:
                // 16-bit mono formats
                img = cv::Mat(stImageInfo.stFrameInfo.nHeight, stImageInfo.stFrameInfo.nWidth, CV_16UC1);
                memcpy(img.data, stImageInfo.pBufAddr, stImageInfo.stFrameInfo.nHeight * stImageInfo.stFrameInfo.nWidth * 2);
                // Normalize to 8-bit for display if needed
                // cv::Mat img8bit;
                // img.convertTo(img8bit, CV_8UC1, 1.0/256.0);
                // img = img8bit;
                break;
                
            case PixelType_Gvsp_Mono10_Packed:
            case PixelType_Gvsp_Mono12_Packed:
                // Handle packed formats - need custom unpacking
                {
                    // For packed formats, we need to unpack to 16-bit
                    img = cv::Mat(stImageInfo.stFrameInfo.nHeight, stImageInfo.stFrameInfo.nWidth, CV_16UC1, cv::Scalar(0));
                    
                    // Unpack based on the format (10-bit or 12-bit)
                    int bitDepth = (stImageInfo.stFrameInfo.enPixelType == PixelType_Gvsp_Mono10_Packed) ? 10 : 12;
                    
                    // Custom unpacking algorithm for packed data
                    // This is a placeholder - actual implementation would depend on the exact packing format
                    std::cerr << "[CameraGetFrame] : Packed format handling needs implementation for " << bitDepth << "-bit data" << std::endl;
                    
                    // Convert to 8-bit for display
                    cv::Mat img8bit;
                    img.convertTo(img8bit, CV_8UC1, 1.0/(1<<(bitDepth-8)));
                    img = img8bit;
                }
                break;

            // RGB formats
            case PixelType_Gvsp_RGB8_Packed:
            {
                img = cv::Mat(stImageInfo.stFrameInfo.nHeight, stImageInfo.stFrameInfo.nWidth, CV_8UC3);
                memcpy(img.data, stImageInfo.pBufAddr, stImageInfo.stFrameInfo.nHeight * stImageInfo.stFrameInfo.nWidth * 3);
                cv::cvtColor(img, img, cv::COLOR_RGB2BGR); // Convert from RGB to BGR for OpenCV
                break;
            }
            case PixelType_Gvsp_BGR8_Packed:
                img = cv::Mat(stImageInfo.stFrameInfo.nHeight, stImageInfo.stFrameInfo.nWidth, CV_8UC3);
                memcpy(img.data, stImageInfo.pBufAddr, stImageInfo.stFrameInfo.nHeight * stImageInfo.stFrameInfo.nWidth * 3);
                // Already in BGR format, no conversion needed
                break;
            
            case PixelType_Gvsp_RGBA8_Packed:
                img = cv::Mat(stImageInfo.stFrameInfo.nHeight, stImageInfo.stFrameInfo.nWidth, CV_8UC4);
                memcpy(img.data, stImageInfo.pBufAddr, stImageInfo.stFrameInfo.nHeight * stImageInfo.stFrameInfo.nWidth * 4);
                cv::cvtColor(img, img, cv::COLOR_RGBA2BGR); // Convert from RGBA to BGR
                break;

            case PixelType_Gvsp_BGRA8_Packed:
                img = cv::Mat(stImageInfo.stFrameInfo.nHeight, stImageInfo.stFrameInfo.nWidth, CV_8UC4);
                memcpy(img.data, stImageInfo.pBufAddr, stImageInfo.stFrameInfo.nHeight * stImageInfo.stFrameInfo.nWidth * 4);
                cv::cvtColor(img, img, cv::COLOR_BGRA2BGR); // Convert from BGRA to BGR
                break;

            case PixelType_Gvsp_RGB10_Packed:
            case PixelType_Gvsp_RGB12_Packed:
            case PixelType_Gvsp_RGB16_Packed:
            case PixelType_Gvsp_BGR10_Packed:
            case PixelType_Gvsp_BGR12_Packed:
            case PixelType_Gvsp_BGR16_Packed:
                // Handle higher bit depth RGB/BGR formats
                {
                    int channels = 3;
                    int depth = (stImageInfo.stFrameInfo.enPixelType == PixelType_Gvsp_RGB10_Packed || 
                                stImageInfo.stFrameInfo.enPixelType == PixelType_Gvsp_BGR10_Packed) ? 10 :
                               (stImageInfo.stFrameInfo.enPixelType == PixelType_Gvsp_RGB12_Packed || 
                                stImageInfo.stFrameInfo.enPixelType == PixelType_Gvsp_BGR12_Packed) ? 12 : 16;
                    
                    // Create a 16-bit per channel image
                    img = cv::Mat(stImageInfo.stFrameInfo.nHeight, stImageInfo.stFrameInfo.nWidth, CV_16UC3);
                    memcpy(img.data, stImageInfo.pBufAddr, stImageInfo.stFrameInfo.nHeight * stImageInfo.stFrameInfo.nWidth * channels * 2);
                    
                    // Convert to 8-bit per channel for display and processing
                    cv::Mat img8bit;
                    img.convertTo(img8bit, CV_8UC3, 1.0/(1<<(depth-8)));
                    
                    // Convert color space if needed
                    if (stImageInfo.stFrameInfo.enPixelType == PixelType_Gvsp_RGB10_Packed ||
                        stImageInfo.stFrameInfo.enPixelType == PixelType_Gvsp_RGB12_Packed ||
                        stImageInfo.stFrameInfo.enPixelType == PixelType_Gvsp_RGB16_Packed) {
                        cv::cvtColor(img8bit, img8bit, cv::COLOR_RGB2BGR);
                    }
                    
                    img = img8bit;
                }
                break;

            // Bayer formats - common in raw camera output
            case PixelType_Gvsp_BayerGR8:
                img = cv::Mat(stImageInfo.stFrameInfo.nHeight, stImageInfo.stFrameInfo.nWidth, CV_8UC1);
                memcpy(img.data, stImageInfo.pBufAddr, stImageInfo.stFrameInfo.nHeight * stImageInfo.stFrameInfo.nWidth);
                cv::cvtColor(img, img, cv::COLOR_BayerGR2BGR);
                break;
// ******************************************************************************************************************************************************//
            case PixelType_Gvsp_BayerRG8:
            {   img = cv::Mat(stImageInfo.stFrameInfo.nHeight, stImageInfo.stFrameInfo.nWidth, CV_8UC1);
                memcpy(img.data, stImageInfo.pBufAddr, stImageInfo.stFrameInfo.nHeight * stImageInfo.stFrameInfo.nWidth);
                cv::cvtColor(img, img, cv::COLOR_BayerRG2RGB);
                break;
            }
                
            case PixelType_Gvsp_BayerGB8:
                img = cv::Mat(stImageInfo.stFrameInfo.nHeight, stImageInfo.stFrameInfo.nWidth, CV_8UC1);
                memcpy(img.data, stImageInfo.pBufAddr, stImageInfo.stFrameInfo.nHeight * stImageInfo.stFrameInfo.nWidth);
                cv::cvtColor(img, img, cv::COLOR_BayerGB2BGR);
                break;
                
            case PixelType_Gvsp_BayerBG8:
                img = cv::Mat(stImageInfo.stFrameInfo.nHeight, stImageInfo.stFrameInfo.nWidth, CV_8UC1);
                memcpy(img.data, stImageInfo.pBufAddr, stImageInfo.stFrameInfo.nHeight * stImageInfo.stFrameInfo.nWidth);
                cv::cvtColor(img, img, cv::COLOR_BayerBG2BGR);
                break;
                
            // Higher bit depth Bayer formats
            case PixelType_Gvsp_BayerGR10:
            case PixelType_Gvsp_BayerRG10:
            case PixelType_Gvsp_BayerGB10:
            case PixelType_Gvsp_BayerBG10:
            case PixelType_Gvsp_BayerGR12:
            case PixelType_Gvsp_BayerRG12:
            case PixelType_Gvsp_BayerGB12:
            case PixelType_Gvsp_BayerBG12:
            case PixelType_Gvsp_BayerGR16:
            case PixelType_Gvsp_BayerRG16:
            case PixelType_Gvsp_BayerGB16:
            case PixelType_Gvsp_BayerBG16:
                {
                    // Determine bit depth
                    int depth = (stImageInfo.stFrameInfo.enPixelType >= PixelType_Gvsp_BayerGR10 && 
                                stImageInfo.stFrameInfo.enPixelType <= PixelType_Gvsp_BayerBG10) ? 10 :
                               (stImageInfo.stFrameInfo.enPixelType >= PixelType_Gvsp_BayerGR12 && 
                                stImageInfo.stFrameInfo.enPixelType <= PixelType_Gvsp_BayerBG12) ? 12 : 16;
                    
                    // Create a 16-bit image to hold the data
                    img = cv::Mat(stImageInfo.stFrameInfo.nHeight, stImageInfo.stFrameInfo.nWidth, CV_16UC1);
                    memcpy(img.data, stImageInfo.pBufAddr, stImageInfo.stFrameInfo.nHeight * stImageInfo.stFrameInfo.nWidth * 2);
                    
                    cv::Mat img8bit;
                    img.convertTo(img8bit, CV_8UC1, 1.0/(1<<(depth-8)));
                    
                    // Determine the Bayer pattern and convert
                    int bayerPattern = -1;
                    if (stImageInfo.stFrameInfo.enPixelType == PixelType_Gvsp_BayerGR10 || 
                        stImageInfo.stFrameInfo.enPixelType == PixelType_Gvsp_BayerGR12 ||
                        stImageInfo.stFrameInfo.enPixelType == PixelType_Gvsp_BayerGR16) {
                        bayerPattern = cv::COLOR_BayerGR2BGR;
                    } else if (stImageInfo.stFrameInfo.enPixelType == PixelType_Gvsp_BayerRG10 || 
                              stImageInfo.stFrameInfo.enPixelType == PixelType_Gvsp_BayerRG12 ||
                              stImageInfo.stFrameInfo.enPixelType == PixelType_Gvsp_BayerRG16) {
                        bayerPattern = cv::COLOR_BayerRG2BGR;
                    } else if (stImageInfo.stFrameInfo.enPixelType == PixelType_Gvsp_BayerGB10 || 
                              stImageInfo.stFrameInfo.enPixelType == PixelType_Gvsp_BayerGB12 ||
                              stImageInfo.stFrameInfo.enPixelType == PixelType_Gvsp_BayerGB16) {
                        bayerPattern = cv::COLOR_BayerGB2BGR;
                    } else {
                        bayerPattern = cv::COLOR_BayerBG2BGR;
                    }
                    
                    cv::cvtColor(img8bit, img, bayerPattern);
                }
                break;
                
            // YUV/YCbCr formats
            case PixelType_Gvsp_YUV422_Packed:
            case PixelType_Gvsp_YUV422_YUYV_Packed:
                {
                    // YUV422 is packed as [Y0 U0 Y1 V0], [Y2 U1 Y3 V1], ...
                    // Each 4 bytes contain data for 2 pixels
                    int numPixels = stImageInfo.stFrameInfo.nWidth * stImageInfo.stFrameInfo.nHeight;
                    cv::Mat yuv(stImageInfo.stFrameInfo.nHeight, stImageInfo.stFrameInfo.nWidth, CV_8UC2);
                    memcpy(yuv.data, stImageInfo.pBufAddr, numPixels * 2); // 2 bytes per pixel for YUV422
                    
                    // Convert YUV to BGR
                    cv::cvtColor(yuv, img, cv::COLOR_YUV2BGR_YUYV);
                }
                break;
                
            case PixelType_Gvsp_YUV444_Packed:
                {
                    // YUV444 has a full Y, U, and V component for each pixel
                    cv::Mat yuv(stImageInfo.stFrameInfo.nHeight, stImageInfo.stFrameInfo.nWidth, CV_8UC3);
                    memcpy(yuv.data, stImageInfo.pBufAddr, stImageInfo.stFrameInfo.nHeight * stImageInfo.stFrameInfo.nWidth * 3);
                    
                    // Convert YUV to BGR
                    cv::cvtColor(yuv, img, cv::COLOR_YUV2BGR);
                }
                break;

            // Custom compressed formats
            case PixelType_Gvsp_Jpeg:
                {
                    // For JPEG format, we need to decode the data
                    std::vector<uchar> jpegData(
                        static_cast<uchar*>(stImageInfo.pBufAddr),
                        static_cast<uchar*>(stImageInfo.pBufAddr) + stImageInfo.stFrameInfo.nFrameLen
                    );
                    
                    img = cv::imdecode(jpegData, cv::IMREAD_COLOR);
                    if (img.empty()) {
                        std::cerr << "[CameraGetFrame] : Failed to decode JPEG data" << std::endl;
                    }
                }
                break;

            // Special cases for the custom HB formats
            case PixelType_Gvsp_HB_Mono8:
            case PixelType_Gvsp_HB_BayerRG8:
            case PixelType_Gvsp_HB_BayerGB8:
            case PixelType_Gvsp_HB_BayerGR8:
            case PixelType_Gvsp_HB_BayerBG8:
            case PixelType_Gvsp_HB_RGB8_Packed:
            case PixelType_Gvsp_HB_BGR8_Packed:
                // Handle custom HB formats - typically similar to standard formats
                // but might have vendor-specific processing
                std::cerr << "[CameraGetFrame] : HB format detected, treating as standard equivalent." << std::endl;
                // Recursively call with the equivalent standard format
                {
                    MV_FRAME_OUT_INFO_EX modifiedFrameInfo = stImageInfo.stFrameInfo;
                    
                    // Map HB format to standard format
                    if (stImageInfo.stFrameInfo.enPixelType == PixelType_Gvsp_HB_Mono8)
                        modifiedFrameInfo.enPixelType = PixelType_Gvsp_Mono8;
                    else if (stImageInfo.stFrameInfo.enPixelType == PixelType_Gvsp_HB_BayerRG8)
                        modifiedFrameInfo.enPixelType = PixelType_Gvsp_BayerRG8;
                    else if (stImageInfo.stFrameInfo.enPixelType == PixelType_Gvsp_HB_BayerGB8)
                        modifiedFrameInfo.enPixelType = PixelType_Gvsp_BayerGB8;
                    else if (stImageInfo.stFrameInfo.enPixelType == PixelType_Gvsp_HB_BayerGR8)
                        modifiedFrameInfo.enPixelType = PixelType_Gvsp_BayerGR8;
                    else if (stImageInfo.stFrameInfo.enPixelType == PixelType_Gvsp_HB_BayerBG8)
                        modifiedFrameInfo.enPixelType = PixelType_Gvsp_BayerBG8;
                    else if (stImageInfo.stFrameInfo.enPixelType == PixelType_Gvsp_HB_RGB8_Packed)
                        modifiedFrameInfo.enPixelType = PixelType_Gvsp_RGB8_Packed;
                    else if (stImageInfo.stFrameInfo.enPixelType == PixelType_Gvsp_HB_BGR8_Packed)
                        modifiedFrameInfo.enPixelType = PixelType_Gvsp_BGR8_Packed;
                    
                    // Create a temporary copy of the image info with modified pixel type
                    MV_FRAME_OUT tempImageInfo = stImageInfo;
                    tempImageInfo.stFrameInfo = modifiedFrameInfo;
                    
                    // Process using the standard format handler
                    // This would involve duplicating the switch logic, which isn't ideal
                    // In a real implementation, you might refactor the pixel type handling into a separate function
                }
                break;

            // YUV420 formats (common in video compression)
            case PixelType_Gvsp_YUV420SP_NV12:
                {
                    // NV12: Y plane followed by interleaved UV plane (at half resolution)
                    cv::Mat nv12(stImageInfo.stFrameInfo.nHeight * 3/2, stImageInfo.stFrameInfo.nWidth, CV_8UC1);
                    memcpy(nv12.data, stImageInfo.pBufAddr, stImageInfo.stFrameInfo.nHeight * stImageInfo.stFrameInfo.nWidth * 3/2);
                    
                    cv::cvtColor(nv12, img, cv::COLOR_YUV2BGR_NV12);
                }
                break;
                
            case PixelType_Gvsp_YUV420SP_NV21:
                {
                    // NV21: Y plane followed by interleaved VU plane (at half resolution)
                    cv::Mat nv21(stImageInfo.stFrameInfo.nHeight * 3/2, stImageInfo.stFrameInfo.nWidth, CV_8UC1);
                    memcpy(nv21.data, stImageInfo.pBufAddr, stImageInfo.stFrameInfo.nHeight * stImageInfo.stFrameInfo.nWidth * 3/2);
                    
                    cv::cvtColor(nv21, img, cv::COLOR_YUV2BGR_NV21);
                }
                break;

            default:
                std::cerr << "[CameraGetFrame] : Unsupported pixel format! PixelType = " << stImageInfo.stFrameInfo.enPixelType << std::endl;
                // Attempt a generic approach for unknown formats
                if ((stImageInfo.stFrameInfo.enPixelType & MV_GVSP_PIX_MONO) == MV_GVSP_PIX_MONO) {
                    // It's some kind of monochrome format
                    int bits = ((stImageInfo.stFrameInfo.enPixelType & MV_GVSP_PIX_EFFECTIVE_PIXEL_SIZE_MASK) >> MV_GVSP_PIX_EFFECTIVE_PIXEL_SIZE_SHIFT);
                    std::cout << "[CameraGetFrame] : Trying generic mono format with " << bits << " bits" << std::endl;
                    
                    if (bits <= 8) {
                        img = cv::Mat(stImageInfo.stFrameInfo.nHeight, stImageInfo.stFrameInfo.nWidth, CV_8UC1);
                        memcpy(img.data, stImageInfo.pBufAddr, stImageInfo.stFrameInfo.nHeight * stImageInfo.stFrameInfo.nWidth);
                    } else {
                        img = cv::Mat(stImageInfo.stFrameInfo.nHeight, stImageInfo.stFrameInfo.nWidth, CV_16UC1);
                        memcpy(img.data, stImageInfo.pBufAddr, stImageInfo.stFrameInfo.nHeight * stImageInfo.stFrameInfo.nWidth * 2);
                        
                        // Convert to 8-bit for display
                        cv::Mat img8bit;
                        img.convertTo(img8bit, CV_8UC1, 1.0/(1<<(bits-8)));
                        img = img8bit;
                    }
                } else if ((stImageInfo.stFrameInfo.enPixelType & MV_GVSP_PIX_COLOR) == MV_GVSP_PIX_COLOR) {
                    // It's some kind of color format
                    std::cerr << "[CameraGetFrame] : Unknown color format - attempting raw copy" << std::endl;
                    img = cv::Mat(stImageInfo.stFrameInfo.nHeight, stImageInfo.stFrameInfo.nWidth, CV_8UC3);
                    memcpy(img.data, stImageInfo.pBufAddr, stImageInfo.stFrameInfo.nHeight * stImageInfo.stFrameInfo.nWidth * 3);
                } else {
                    std::cerr << "[CameraGetFrame] : Cannot determine format type" << std::endl;
                MV_CC_FreeImageBuffer(handle_, &stImageInfo);
                return;
                }
        }

        // Free image buffer after processing
        nRet_ = MV_CC_FreeImageBuffer(handle_, &stImageInfo);
        if (nRet_ != MV_OK) {
            std::cerr << "[CameraGetFrame] : Free Image Buffer fail! nRet=" << nRet_ << std::endl;
        }
    } else {
        std::cerr << "[CameraGetFrame] : Failed to get frame! nRet=" << nRet_ << std::endl;
    }
}




Camera::Camera (const std::string initFilePath) : camera_is_open_(false) {

    init_file_path_ = initFilePath;

    // 打开 YAML 文件
    cv::FileStorage fs(init_file_path_, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "[CameraInit] : Failed to open " << init_file_path_ << std::endl;
        return;
    }

    fs["camera_type"] >> camera_type_;

    if (camera_type_ == USB) {
        fs["base_info"]["width"] >> usb_init_.width;  // Changed from "weight" to "width"
        fs["base_info"]["height"] >> usb_init_.height;
        fs["base_info"]["fps"] >> usb_init_.fps;
        fs["pixel_format"] >> usb_init_.video_capture_api;
        fs["index"] >> usb_init_.index;
        usb_init_.fourcc.clear();
        for (const auto &node: fs["fourcc"]) {
            std::string ch = node;
            usb_init_.fourcc += ch;
        }

    } else if (camera_type_ == HIK) {
        fs["base_info"]["width"] >> hik_init_.width;  // Changed from "weight" to "width"
        fs["base_info"]["height"] >> hik_init_.height;
        fs["base_info"]["fps"] >> hik_init_.fps;
        fs["hik_info"]["camera_mode"] >> hik_init_.camera_mode;
        fs["hik_info"]["exposure_time"] >> hik_init_.exposure_time;
        fs["pixel_format"] >> hik_init_.video_capture_api;

    } else {
        std::cerr << "[CameraInit] : Unexpected Camera Type " << std::endl;
        return;
    }
    // 读取 camera_matrix
    fs["camera_matrix"] >> camera_matrix_;
    // 读取 dist_coeffs_matrix
    fs["dist_coeffs_matrix"] >> dist_coeffs_matrix_;
    // 读取 "pos" 数组
    cv::FileNode RTNode = fs["RT"];
    if (RTNode.empty()) {
        std::cerr << "[CameraInit] : Failed to find RT" << std::endl;
        return;
    }
    for (const auto& node : RTNode) {
        if (node["yaw"].empty()) {
            // 读取坐标 x, y, z
            coordinate_[0] = static_cast<float>((int)node["x"]);
            coordinate_[1] = static_cast<float>((int)node["y"]);
            coordinate_[2] = static_cast<float>((int)node["z"]);
        } else {
            // 读取欧拉角 yaw, pitch, roll
            euler_angles_[0] = static_cast<float>((int)node["yaw"]);
            euler_angles_[1] = static_cast<float>((int)node["pitch"]);
            euler_angles_[2] = static_cast<float>((int)node["roll"]);
        }
    }
    // 关闭文件
    fs.release();
    //打开相机
    open();
}

Camera::~Camera() {
    stop();
}

bool Camera::getFrame(cv::Mat &img) {

    if (!camera_is_open_) {
        std::cerr << "[CameraGetFrame] : Camera has not opened! " << std::endl;
        return false;
    }
    if (camera_type_ == USB) {
        cap_->read(img);
        return true;
    } else if (camera_type_ == HIK) {
        hik_cam_->getFrame(img);
        return true;
    } else {
        std::cerr << "[CameraGetFrame] : Unexpected Camera Type " << std::endl;
        return false;
    }
}

double Camera::cameraCalibrate(const std::string& imgPath, int board_width, int board_height, float square_size) {
    cv::Size board_size(board_width, board_height);

    // 准备棋盘格的 3D 世界坐标点（单位：厘米）
    std::vector<cv::Point3f> object_points;
    object_points.reserve(board_width * board_height);
    for (int i = 0; i < board_height; i++) {
        for (int j = 0; j < board_width; j++) {
            object_points.push_back(cv::Point3f(j * square_size, i * square_size, 0));
        }
    }

    std::vector<std::vector<cv::Point3f>> object_points_all;
    std::vector<std::vector<cv::Point2f>> image_points_all;

    // 读取标定图像
    std::vector<cv::String> images;
    cv::glob(imgPath + "*.jpg", images);
    images.erase(std::remove_if(images.begin(), images.end(),
                               [](const cv::String& filename) { return filename.find("DS_Store") != std::string::npos; }),
                images.end());

    for (size_t i = 0; i < images.size(); i++) {
        cv::Mat image = cv::imread(images[i]);
        if (image.empty()) {
            std::cout << "Failed to read image: " << images[i] << std::endl;
            continue;
        }

        cv::Mat gray;
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
        cv::equalizeHist(gray, gray);  // 自适应直方图均衡化

        std::vector<cv::Point2f> corners;
        bool found = cv::findChessboardCorners(gray, board_size, corners);

        if (found) {
            cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1), 
                             cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 30, 0.1));

            image_points_all.push_back(corners);
            object_points_all.push_back(object_points);

            cv::drawChessboardCorners(image, board_size, corners, found);
            cv::imshow("Chessboard", image);
            cv::waitKey(1);
        } else {
            std::cout << "Chessboard corners not found in image: " << images[i] << std::endl;
        }
    }

    cv::Mat camera_matrix = cv::Mat::eye(3, 3, CV_64F); // 相机内参矩阵
    cv::Mat dist_coeffs; // 畸变系数
    std::vector<cv::Mat> rvecs, tvecs; // 旋转和平移向量

    cv::Size image_size = cv::imread(images[0]).size();
    double rms = cv::calibrateCamera(object_points_all, image_points_all, image_size,
                                     camera_matrix, dist_coeffs, rvecs, tvecs);

    std::cout << "Reprojection error: " << rms << std::endl;
    std::cout << "Camera Matrix:\n" << camera_matrix << std::endl;
    std::cout << "Distortion Coefficients:\n" << dist_coeffs << std::endl;

    cv::destroyAllWindows();

    return rms;
}


bool Camera::isOpened() {
    return camera_is_open_;
}

int Camera::getFPS() {
    if (camera_type_ == USB) {
        return usb_init_.fps;
    } else if (camera_type_ == HIK) {
        return hik_init_.fps;
    }
}

void Camera::open() {
    if (camera_is_open_) return;
    if (camera_type_ == USB) {
        cap_ = new cv::VideoCapture(usb_init_.index, usb_init_.video_capture_api);
        cap_->set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc(
                usb_init_.fourcc[0], usb_init_.fourcc[1], usb_init_.fourcc[2], usb_init_.fourcc[3]));
        cap_->set(cv::CAP_PROP_FRAME_WIDTH, usb_init_.width);  // Changed from "weight" to "width"
        cap_->set(cv::CAP_PROP_FRAME_HEIGHT, usb_init_.height);
        cap_->set(cv::CAP_PROP_FPS, usb_init_.fps);
        if (cap_->isOpened()) camera_is_open_ = true;
    } else if (camera_type_ == HIK) {
        hik_cam_ = new HK_Camera();
        camera_is_open_ = hik_cam_->cameraInit(hik_init_);
        if (camera_is_open_) {
            hik_cam_->startCamera();
        }else{
            std::cout << "open false" << std::endl;
        }
    }
}

void Camera::stop() {
    if (!camera_is_open_) return; 

    if (camera_type_ == USB) {
        delete cap_;
        camera_is_open_ = false;
    } else if (camera_is_open_ == HIK) {
        hik_cam_->stopCamera();
        delete hik_cam_;
        camera_is_open_ = false;
    }
}
