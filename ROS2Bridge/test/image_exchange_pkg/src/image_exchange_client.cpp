#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <itom_ros2_test/srv/exchange2_d_array.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

class ImageExchangeClient : public rclcpp::Node
{
public:
    ImageExchangeClient() : Node("image_exchange_client")
    {
        client_ = this->create_client<itom_ros2_test::srv::Exchange2DArray>("image_exchange");

        // Wait for the service to be available
        while (!client_->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
        }

        // Send a request
        send_image_request();
    }

private:
    void send_image_request()
    {
        std::string package_share_directory = ament_index_cpp::get_package_share_directory("image_exchange_pkg");
        std::string image_path = package_share_directory + "/data/Software_solutions.jpg";
        // Load the test image
        cv::Mat cv_image = cv::imread(image_path, cv::IMREAD_COLOR);
        if (cv_image.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to load test image.");
            return;
        }

        // Convert to ROS image message
        auto ros_image_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", cv_image).toImageMsg();

        // Create a service request
        auto request = std::make_shared<itom_ros2_test::srv::Exchange2DArray::Request>();
        request->request_image = *ros_image_msg;

RCLCPP_INFO(this->get_logger(), "Sending image request...");

// 2. Asynchronously call async_send_request with a Lambda callback
client_->async_send_request(
    request,
    // This is an asynchronous callback. When the server responds, the following Lambda will be called.
    [this](rclcpp::Client<itom_ros2_test::srv::Exchange2DArray>::SharedFuture response_future)
    {
        // Remember to use try-catch in the callback to ensure that the log can be printed when an error occurs.
        try
        {
            auto response = response_future.get();
            RCLCPP_INFO(this->get_logger(), "Received response image.");

            // Save the returned image locally
            cv::Mat response_image = cv_bridge::toCvCopy(response->response_image, "bgr8")->image;
            cv::imwrite("response_image.jpg", response_image);
            RCLCPP_INFO(this->get_logger(), "Response image saved as response_image.jpg.");
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
        }
    }
);
    }

    rclcpp::Client<itom_ros2_test::srv::Exchange2DArray>::SharedPtr client_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageExchangeClient>());
    rclcpp::shutdown();
    return 0;
}
