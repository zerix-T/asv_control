#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64.hpp"

class CmdVelToThrusters : public rclcpp::Node
{
public:
    CmdVelToThrusters()
    : Node("cmd_vel_to_thrusters")
    {
        // Parameters
        this->declare_parameter("thruster_spacing", 1.0);   // meters
        this->declare_parameter("thrust_scale", 100.0);      // scale factor

        thruster_spacing_ = this->get_parameter("thruster_spacing").as_double();
        thrust_scale_ = this->get_parameter("thrust_scale").as_double();

        // Publishers
        left_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "/wamv/thrusters/left/thrust", 10);

        right_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "/wamv/thrusters/right/thrust", 10);

        // Subscriber
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel",
            10,
            std::bind(&CmdVelToThrusters::cmdVelCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "cmd_vel_to_thrusters node started");
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        double linear = msg->linear.x;
        double angular = msg->angular.z;

        // Differential thrust mixing
        double left =
            linear - (angular * thruster_spacing_ / 2.0);

        double right =
            linear + (angular * thruster_spacing_ / 2.0);

        std_msgs::msg::Float64 left_msg;
        std_msgs::msg::Float64 right_msg;

        left_msg.data = left * thrust_scale_;
        right_msg.data = right * thrust_scale_;

        left_pub_->publish(left_msg);
        right_pub_->publish(right_msg);

        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "Left: %.2f  Right: %.2f",
            left_msg.data,
            right_msg.data);
    }

    double thruster_spacing_;
    double thrust_scale_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_pub_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CmdVelToThrusters>());
    rclcpp::shutdown();
    return 0;
}