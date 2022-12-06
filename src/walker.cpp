// Copyright(c) 2022 Jay Prajapati
//
// Permission is hereby granted,
// free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and / or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :
//
// The above copyright notice and this permission notice shall be included in all copies
// or
// substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS",
// WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
// DAMAGES OR OTHER
// LIABILITY,
// WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

/**
 * @file walker.cpp
 * @author Jay Prajapati (jayp@umd.edu)
 * @brief Initiates the walker node and implements the walker algorithm
 * @version 0.1
 * @date 2022-12-06
 *
 * @copyright Copyright (c) 2022
 *
 */
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals;

using LASER_SCAN = sensor_msgs::msg::LaserScan;
using TWIST = geometry_msgs::msg::Twist;

/**
 * @brief class stub that initiates the walker node. In addition gets the laser scan data and declares the methods to moce the robot accordingly.
 *
 */
class Walker : public rclcpp::Node
{
public:
    /**
     * @brief Construct a new Walker object
     *
     */
    Walker() : Node("walker")
    {
        auto callback = std::bind(&Walker::laserData_callback, this, _1);
        laser_data_sub = this->create_subscription<LASER_SCAN>("scan", 10, callback);
        publish_velocity = this->create_publisher<TWIST>("cmd_vel", 10);
    }

private:
    /**
     * @brief Gets the lasre scan data
     *
     * @param scanData
     */
    void laserData_callback(const LASER_SCAN &scanData)
    {
        if (scanData.header.stamp.sec == 0)
        {
            return;
        }

        auto laser_scan_data = scanData.ranges;
        for (int i = 330; i < 330 + 60; i++)
        {
            if (laser_scan_data[i % 360] < 0.8)
            {
                move_robot(0.0, 0.1);
            }
            else
            {
                move_robot(0.1, 0.0);
            }
        }
    }

    /**
     * @brief gives the commands to move the robot as per the laser scan data
     *
     * @param x_velocity
     * @param z_velocity
     */
    void move_robot(float x_velocity, float z_velocity)
    {
        auto velocity_msg = TWIST();
        velocity_msg.linear.x = x_velocity;
        velocity_msg.angular.z = -z_velocity;
        publish_velocity->publish(velocity_msg);
    }

    // Declaring the private variables
    rclcpp::Subscription<LASER_SCAN>::SharedPtr laser_data_sub;
    rclcpp::Publisher<TWIST>::SharedPtr publish_velocity;
    rclcpp::TimerBase::SharedPtr timer;
    LASER_SCAN last_scan_data;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Walker>());
    rclcpp::shutdown();
    return 0;
}