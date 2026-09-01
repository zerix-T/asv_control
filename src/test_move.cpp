#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace std::chrono_literals;

class ForwardTest : public rclcpp::Node
{
public:
    ForwardTest() : Node("forward_test")
    {
        left_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "/wamv/thrusters/left/thrust", 10);

        right_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "/wamv/thrusters/right/thrust", 10);

        timer_ = this->create_wall_timer(
            100ms,
            std::bind(&ForwardTest::timerCallback, this));

        start_time_ = this->now();

        RCLCPP_INFO(this->get_logger(), "Publishing forward thrust for 5 seconds...");
    }

private:
    void timerCallback()
    {
        std_msgs::msg::Float64 left;
        std_msgs::msg::Float64 right;

        auto elapsed = (this->now() - start_time_).seconds();

        if (elapsed < 10.0)
        {
            left.data = 900.0;
            right.data = 1200.0;
        }
        else
        {
            left.data = 0.0;
            right.data = 0.0;
        }

        left_pub_->publish(left);
        right_pub_->publish(right);

        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "Left: %.1f  Right: %.1f",
            left.data,
            right.data);
    }

    rclcpp::Time start_time_;

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ForwardTest>());
    rclcpp::shutdown();
    return 0;
}