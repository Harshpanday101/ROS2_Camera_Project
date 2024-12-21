#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>

class ImageConversionNode : public rclcpp::Node {
public:
    ImageConversionNode() : Node("image_conversion_node"), mode_(2) {
        // Declare parameters
        this->declare_parameter<std::string>("input_topic", "/camera/image_raw");
        this->declare_parameter<std::string>("output_topic", "/image_converted");

        // Get parameter values
        input_topic_ = this->get_parameter("input_topic").as_string();
        output_topic_ = this->get_parameter("output_topic").as_string();

        // Create subscriber and publisher
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            input_topic_, 10,
            std::bind(&ImageConversionNode::imageCallback, this, std::placeholders::_1));
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(output_topic_, 10);

        // Create service to toggle modes
        mode_service_ = this->create_service<std_srvs::srv::SetBool>(
            "set_mode", std::bind(&ImageConversionNode::setModeCallback, this, std::placeholders::_1, std::placeholders::_2));
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        cv::Mat cv_image = cv_bridge::toCvShare(msg, "bgr8")->image;

        if (mode_ == 1) {
            cv::cvtColor(cv_image, cv_image, cv::COLOR_BGR2GRAY);
        }

        auto converted_msg = cv_bridge::CvImage(msg->header, (mode_ == 1) ? "mono8" : "bgr8", cv_image).toImageMsg();
        image_pub_->publish(*converted_msg);
    }

    void setModeCallback(const std_srvs::srv::SetBool::Request::SharedPtr request,
                         std_srvs::srv::SetBool::Response::SharedPtr response) {
        mode_ = request->data ? 1 : 2;
        response->success = true;
        response->message = "Mode changed successfully!";
    }

    int mode_;
    std::string input_topic_, output_topic_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr mode_service_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageConversionNode>());
    rclcpp::shutdown();
    return 0;
}
