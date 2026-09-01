/**
 * @file laserscan_to_obstacledist.hpp
 * @author Manish Kumar Gupta <mformanish6@gmail.com>
 * @copyright 2026 Enerbots Lab
 * @license MIT
 * @brief laserscan_to_obstacledist subscribe the LaserScan msg and convert it to Mavlink Obstacle_Distance msg and publish it to serial port
 */

#ifndef LASERSCAN_TO_OBSTACLEDIST_HPP
#define LASERSCAN_TO_OBSTACLEDIST_HPP

#include <chrono>
#include <functional>
#include <memory>
#include <array>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace asv_control
{
    class LaserscanToObstacleDist : public rclcpp::Node
    {
    public:
        explicit LaserscanToObstacleDist(const rclcpp::NodeOptions &options);
        ~LaserscanToObstacleDist();
    
    private:
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
        void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg);
        void processLaserScan(const sensor_msgs::msg::LaserScan::ConstSharedPtr &msg);
        bool initMavLink();
        void sendObstacleDist();

        int uart_fd_{-1};
        std::array<uint16_t, 72> obstacle_distances_{};
    };
} // namespace asv_control

#endif // LASERSCAN_TO_OBSTACLEDIST_HPP