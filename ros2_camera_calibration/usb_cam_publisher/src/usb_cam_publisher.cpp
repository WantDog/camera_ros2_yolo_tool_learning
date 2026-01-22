#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

class USBCamPublisher : public rclcpp::Node
{
public:
    USBCamPublisher()
        : Node("usb_cam_publisher")
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("image_raw", 10);

        // 打开摄像头，使用 V4L2 backend 更稳定
        std::string device = "/dev/video2";  // 根据实际设备修改
        cap_.open(device, cv::CAP_V4L2);

        if (!cap_.isOpened())
        {
            RCLCPP_FATAL(this->get_logger(), "无法打开相机: %s", device.c_str());
            throw std::runtime_error("Camera open failed");
        }

        // 设置分辨率和帧率
        cap_.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
        cap_.set(cv::CAP_PROP_FPS, 30);

        int width = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_WIDTH));
        int height = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_HEIGHT));

        if (width <= 0 || height <= 0)
        {
            RCLCPP_FATAL(this->get_logger(),
                         "非法分辨率: %dx%d", width, height);
            throw std::runtime_error("Invalid image size");
        }

        RCLCPP_INFO(this->get_logger(),
                    "Camera opened: %dx%d", width, height);

        // 定时发布帧
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33),
            std::bind(&USBCamPublisher::publish_frame, this));
    }

private:
    void publish_frame()
    {
        cv::Mat frame;
        cap_ >> frame;

        // 防止 frame 异常
        if (frame.empty() || frame.cols <= 0 || frame.rows <= 0)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "捕获到空帧或非法帧: %dx%d", frame.cols, frame.rows);
            return;
        }

        // 如果摄像头返回 MJPEG 或 YUYV，需要转换到 BGR
        if (frame.channels() != 3)
        {
            cv::cvtColor(frame, frame, cv::COLOR_YUV2BGR_YUYV);
        }

        auto msg = cv_bridge::CvImage(
            std_msgs::msg::Header(),
            "bgr8",
            frame).toImageMsg();

        msg->header.stamp = this->now();
        publisher_->publish(*msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture cap_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    try
    {
        rclcpp::spin(std::make_shared<USBCamPublisher>());
    }
    catch (const std::exception &e)
    {
        RCLCPP_FATAL(rclcpp::get_logger("main"), "%s", e.what());
    }

    rclcpp::shutdown();
    return 0;
}
