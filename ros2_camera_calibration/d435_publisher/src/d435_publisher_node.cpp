#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <camera_info_manager/camera_info_manager.hpp>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <memory>

class D435Publisher : public rclcpp::Node
{
public:
    D435Publisher() : Node("d435_publisher"), pipe_started_(false)
    {
        // 声明参数
        this->declare_parameter("width", 640);
        this->declare_parameter("height", 480);
        this->declare_parameter("fps", 30);
        this->declare_parameter("enable_depth", false);
        this->declare_parameter("camera_info_url", "");
        
        // 获取参数
        width_ = this->get_parameter("width").as_int();
        height_ = this->get_parameter("height").as_int();
        fps_ = this->get_parameter("fps").as_int();
        enable_depth_ = this->get_parameter("enable_depth").as_bool();
        
        // 初始化发布器
        color_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/d435/color/image_raw", 1);
        color_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/d435/color/camera_info", 1);
        
        if (enable_depth_) {
            depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/d435/depth/image_raw", 1);
            depth_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/d435/depth/camera_info", 1);
        }
        
        // 初始化相机信息管理器
        std::string camera_info_url = this->get_parameter("camera_info_url").as_string();
        camera_info_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(
            this, "d435_color", camera_info_url);
        
        // 初始化RealSense管道
        try {
            initializeCamera();
            
            // 创建定时器发布图像
            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(1000 / fps_),
                std::bind(&D435Publisher::publishImages, this));
                
            RCLCPP_INFO(this->get_logger(), "D435 Publisher initialized successfully");
            RCLCPP_INFO(this->get_logger(), "Resolution: %dx%d @ %d fps", width_, height_, fps_);
            RCLCPP_INFO(this->get_logger(), "Depth enabled: %s", enable_depth_ ? "true" : "false");
        }
        catch (const rs2::error& e) {
            RCLCPP_ERROR(this->get_logger(), "RealSense error: %s", e.what());
            rclcpp::shutdown();
        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error: %s", e.what());
            rclcpp::shutdown();
        }
    }
    
    ~D435Publisher()
    {
        try {
            if (pipe_started_) {
                pipe_.stop();
            }
        }
        catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Error stopping pipeline: %s", e.what());
        }
    }

private:
    void initializeCamera()
    {
        // 配置RealSense管道
        rs2::config cfg;
        cfg.enable_stream(RS2_STREAM_COLOR, width_, height_, RS2_FORMAT_BGR8, fps_);
        
        if (enable_depth_) {
            cfg.enable_stream(RS2_STREAM_DEPTH, width_, height_, RS2_FORMAT_Z16, fps_);
        }
        
        // 启动管道
        rs2::pipeline_profile profile = pipe_.start(cfg);
        pipe_started_ = true;
        
        // 获取相机内参
        auto color_stream = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
        auto color_intrinsics = color_stream.get_intrinsics();
        
        // 设置相机信息
        setupCameraInfo(color_intrinsics);
        
        RCLCPP_INFO(this->get_logger(), "Camera intrinsics - fx: %.2f, fy: %.2f, cx: %.2f, cy: %.2f",
                    color_intrinsics.fx, color_intrinsics.fy, color_intrinsics.ppx, color_intrinsics.ppy);
    }
    
    void setupCameraInfo(const rs2_intrinsics& intrinsics)
    {
        camera_info_msg_.header.frame_id = "d435_color_optical_frame";
        camera_info_msg_.width = intrinsics.width;
        camera_info_msg_.height = intrinsics.height;
        camera_info_msg_.distortion_model = "plumb_bob";
        
        // 相机内参矩阵 K
        camera_info_msg_.k[0] = intrinsics.fx;  // fx
        camera_info_msg_.k[2] = intrinsics.ppx; // cx
        camera_info_msg_.k[4] = intrinsics.fy;  // fy
        camera_info_msg_.k[5] = intrinsics.ppy; // cy
        camera_info_msg_.k[8] = 1.0;
        
        // 投影矩阵 P
        camera_info_msg_.p[0] = intrinsics.fx;  // fx
        camera_info_msg_.p[2] = intrinsics.ppx; // cx
        camera_info_msg_.p[5] = intrinsics.fy;  // fy
        camera_info_msg_.p[6] = intrinsics.ppy; // cy
        camera_info_msg_.p[10] = 1.0;
        
        // 畸变系数 D (Brown-Conrady模型)
        camera_info_msg_.d.resize(5);
        for (int i = 0; i < 5; i++) {
            camera_info_msg_.d[i] = intrinsics.coeffs[i];
        }
        
        // 矫正矩阵 R (单目相机为单位矩阵)
        camera_info_msg_.r[0] = 1.0;
        camera_info_msg_.r[4] = 1.0;
        camera_info_msg_.r[8] = 1.0;
    }
    
    void publishImages()
    {
        try {
            // 等待帧
            rs2::frameset frames = pipe_.wait_for_frames(1000); // 1秒超时
            
            // 获取彩色帧
            rs2::frame color_frame = frames.get_color_frame();
            
            if (color_frame) {
                publishColorImage(color_frame);
            }
            
            // 如果启用深度，发布深度图像
            if (enable_depth_) {
                rs2::depth_frame depth_frame = frames.get_depth_frame();
                if (depth_frame) {
                    publishDepthImage(depth_frame);
                }
            }
        }
        catch (const rs2::error& e) {
            RCLCPP_WARN(this->get_logger(), "RealSense error in publishImages: %s", e.what());
        }
        catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Error in publishImages: %s", e.what());
        }
    }
    
    void publishColorImage(const rs2::frame& color_frame)
    {
        // 转换为OpenCV Mat
        cv::Mat color_image(cv::Size(width_, height_), CV_8UC3, 
                           (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
        
        // 转换为ROS消息
        auto image_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", color_image).toImageMsg();
        
        // 设置时间戳和frame_id
        rclcpp::Time now = this->now();
        image_msg->header.stamp = now;
        image_msg->header.frame_id = "d435_color_optical_frame";
        
        // 发布图像
        color_pub_->publish(*image_msg);
        
        // 发布相机信息
        camera_info_msg_.header.stamp = now;
        color_info_pub_->publish(camera_info_msg_);
    }
    
    void publishDepthImage(const rs2::depth_frame& depth_frame)
    {
        // 转换为OpenCV Mat (16位深度)
        cv::Mat depth_image(cv::Size(width_, height_), CV_16UC1, 
                           (void*)depth_frame.get_data(), cv::Mat::AUTO_STEP);
        
        // 转换为ROS消息
        auto depth_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "16UC1", depth_image).toImageMsg();
        
        // 设置时间戳和frame_id
        rclcpp::Time now = this->now();
        depth_msg->header.stamp = now;
        depth_msg->header.frame_id = "d435_depth_optical_frame";
        
        // 发布深度图像
        depth_pub_->publish(*depth_msg);
        
        // 发布深度相机信息
        auto depth_info = camera_info_msg_;
        depth_info.header.stamp = now;
        depth_info.header.frame_id = "d435_depth_optical_frame";
        depth_info_pub_->publish(depth_info);
    }

private:
    // RealSense相关
    rs2::pipeline pipe_;
    bool pipe_started_;
    
    // ROS2相关
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr color_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr color_info_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr depth_info_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // 相机信息
    std::shared_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
    sensor_msgs::msg::CameraInfo camera_info_msg_;
    
    // 参数
    int width_;
    int height_;
    int fps_;
    bool enable_depth_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<D435Publisher>();
        rclcpp::spin(node);
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("d435_publisher"), "Exception: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}