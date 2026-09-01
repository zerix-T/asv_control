#include "asv_control/laserscan_to_obstacledist.hpp"
#include "mavlink/v2.0/common/mavlink.h"

#include <cstring>
#include <cmath>
#include <cerrno>
#include <limits>

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

namespace asv_control
{
    namespace
    {
        // ---- Tunables -------------------------------------------------------
        constexpr const char *kUartDevice   = "/dev/ttyAMA0";    // UART, e.g. /dev/serial0, /dev/ttyUSB0
        constexpr speed_t     kBaudRate      = B921600;         // Cube Orange TELEM port baud
        constexpr uint8_t     kMavSystemId   = 200;              // companion computer "system" id
        constexpr uint8_t     kMavComponentId= MAV_COMP_ID_OBSTACLE_AVOIDANCE;
        constexpr uint8_t     kSectorCount   = 72;             // MAVLink OBSTACLE_DISTANCE fixed size
        constexpr uint8_t     kSectorIncrDeg = 5;              // 72 * 5 = 360 degrees
        constexpr uint16_t    kUnknownDist   = UINT16_MAX;     // "no valid reading" per MAVLink spec
    } // namespace

    LaserscanToObstacleDist::LaserscanToObstacleDist(const rclcpp::NodeOptions &options): rclcpp::Node("laserscan_to_obstacledist", options)
    {
        RCLCPP_INFO(this->get_logger(), "Starting LaserscanToObstacleDist node...");

        obstacle_distances_.fill(UINT16_MAX);

        if (!initMavLink())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize MAVLink interface.");
        }

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan",
            rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
                scanCallback(msg);
            });

        RCLCPP_INFO(this->get_logger(),"Subscribed to /scan");
    }

    LaserscanToObstacleDist::~LaserscanToObstacleDist()
    {
        if (uart_fd_ >= 0)
        {
            close(uart_fd_);
        }

        RCLCPP_INFO(this->get_logger(),"LaserscanToObstacleDist node stopped.");
    }

    bool LaserscanToObstacleDist::initMavLink()
    {
        uart_fd_ = open(kUartDevice, O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (uart_fd_ < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "open(%s) failed: %s", kUartDevice, std::strerror(errno));
            return false;
        }

        termios tty{};
        if (tcgetattr(uart_fd_, &tty) != 0)
        {
            RCLCPP_ERROR(this->get_logger(), "tcgetattr failed: %s", std::strerror(errno));
            close(uart_fd_);
            uart_fd_ = -1;
            return false;
        }

        cfsetispeed(&tty, kBaudRate);
        cfsetospeed(&tty, kBaudRate);

        tty.c_cflag |= (CLOCAL | CREAD);   // ignore modem ctrl lines, enable receiver
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;                // 8 data bits
        tty.c_cflag &= ~PARENB;            // no parity
        tty.c_cflag &= ~CSTOPB;            // 1 stop bit
        tty.c_cflag &= ~CRTSCTS;           // no hardware flow control

        tty.c_lflag = 0;                  // raw input, no echo/canonical
        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // no software flow control
        tty.c_iflag &= ~(ICRNL | INLCR);
        tty.c_oflag = 0;                  // raw output

        tty.c_cc[VMIN]  = 0;
        tty.c_cc[VTIME] = 0;

        if (tcsetattr(uart_fd_, TCSANOW, &tty) != 0)
        {
            RCLCPP_ERROR(this->get_logger(), "tcsetattr failed: %s", std::strerror(errno));
            close(uart_fd_);
            uart_fd_ = -1;
            return false;
        }

        tcflush(uart_fd_, TCIOFLUSH);

        RCLCPP_INFO(this->get_logger(), "UART %s opened for MAVLink at baud index %d", kUartDevice, (int)kBaudRate);
        return true;
    }

    void LaserscanToObstacleDist::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
    {
        processLaserScan(scan_msg);
        sendObstacleDist();
    }

    void LaserscanToObstacleDist::processLaserScan(const sensor_msgs::msg::LaserScan::ConstSharedPtr &msg)
    {
        // Reset sectors each scan; anything not touched stays "unknown".
        obstacle_distances_.fill(kUnknownDist);

        const float range_min_m = msg->range_min;
        const float range_max_m = msg->range_max;

        for (size_t i = 0; i < msg->ranges.size(); ++i)
        {
            const float r = msg->ranges[i];

            if (!std::isfinite(r) || r < range_min_m || r > range_max_m)
            {
                continue; // invalid / out-of-range reading, leave sector as-is
            }

            // Angle of this beam in radians -> degrees in [0, 360)
            float angle_rad = msg->angle_min + static_cast<float>(i) * msg->angle_increment;
            float angle_deg = angle_rad * 180.0f / static_cast<float>(M_PI);

            angle_deg = std::fmod(angle_deg, 360.0f);
            if (angle_deg < 0.0f)
            {
                angle_deg += 360.0f;
            }

            uint8_t sector = static_cast<uint8_t>(angle_deg / kSectorIncrDeg) % kSectorCount;

            // Range in meters -> centimeters for MAVLink OBSTACLE_DISTANCE
            uint16_t r_cm = static_cast<uint16_t>(std::lround(r * 100.0f));

            // Keep the closest obstacle seen in each sector during this scan.
            if (r_cm < obstacle_distances_[sector])
            {
                obstacle_distances_[sector] = r_cm;
            }
        }
    }

    void LaserscanToObstacleDist::sendObstacleDist()
    {
        if (uart_fd_ < 0)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                  "UART not initialized, dropping OBSTACLE_DISTANCE message");
            return;
        }

        mavlink_message_t msg;
        uint64_t time_usec = static_cast<uint64_t>(this->now().nanoseconds() / 1000);

        // NOTE: min/max distance are in centimeters, per MAVLink spec.
        uint16_t min_dist_cm = 5;    // lidar's real min range
        uint16_t max_dist_cm = 3000;  // lidar's real max range

        mavlink_msg_obstacle_distance_pack(
            kMavSystemId,
            kMavComponentId,
            &msg,
            time_usec,
            MAV_DISTANCE_SENSOR_LASER,
            obstacle_distances_.data(),
            kSectorIncrDeg,   // increment (deg), used when increment_f == 0
            min_dist_cm,
            max_dist_cm,
            0.0f,             // increment_f (0 => use integer 'increment' field above)
            0.0f,             // angle_offset: sector 0 centered on vehicle forward (0 deg)
            MAV_FRAME_BODY_FRD);

        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);

        ssize_t written = write(uart_fd_, buffer, len);
        if (written < 0)
        {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                   "UART write failed: %s", std::strerror(errno));
        }
        else if (written != len)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                  "Partial UART write: %zd of %u bytes", written, len);
        }
    }
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(asv_control::LaserscanToObstacleDist)