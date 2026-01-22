#include<rclcpp/rclcpp.hpp>
#include<sensor_msgs/msg/image.hpp>
#include<opencv2/opencv.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include<cv_bridge/cv_bridge.h>
#include<std_msgs/msg/header.hpp>
#include"camera_driver.h"

class HK_Camera_node : public rclcpp::Node{
public:
    HK_Camera_node() : Node("hk_camera_node")
    {
        image_pub = this->create_publisher<sensor_msgs::msg::Image>("/hk_camera/image_raw", 10);
        camera = new Camera("camera_driver/camera_init/HIKcamera0.yaml");
        if (!camera->isOpened())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open camera");
            return;
        }
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1), // ~30 fps
            std::bind(&HK_Camera_node::publishImage, this));
        }
    ~HK_Camera_node(){
        if(camera)
        {
            camera->stop();
            delete camera;
        }
    }
private:
    void publishImage(){
        cv::Mat frame;
        if (camera->getFrame(frame))
        {
            // 转换为ROS消息
            auto image_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
            image_msg->header.stamp = this->now();
            // 发布图像
            image_pub->publish(*image_msg);
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr came_info_pub;
    rclcpp::TimerBase::SharedPtr timer_;
    Camera *camera;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HK_Camera_node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}