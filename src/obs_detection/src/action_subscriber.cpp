#include <memory>
#include <algorithm>

#include "pid_step.cpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Transform.h"

using std::placeholders::_1;

class ControllerNode : public rclcpp::Node
{
  public:
    ControllerNode() : Node("action_subscriber")
    {
      subscription_1 = this->create_subscription<std_msgs::msg::String>(
      "/action", 10, std::bind(&ControllerNode::topic_callback, this, _1));
      
      subscription_2 = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&ControllerNode::odom_callback, this, _1));

      subscription_3 = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, std::bind(&ControllerNode::scan_callback, this, _1));

      publisher_1 = this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);
      publisher_2 = this->create_publisher<std_msgs::msg::String>("/action", 10);

    }

    nav_msgs::msg::Odometry::SharedPtr robot;
    std::string current_action;

    void avoid_obs(float meas, bool right)
    {
      // keep record of last heading
      tf2::Quaternion q(robot->pose.pose.orientation.x, robot->pose.pose.orientation.y,
        robot->pose.pose.orientation.z, robot->pose.pose.orientation.w);
      tf2::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);

      RCLCPP_INFO(this->get_logger(), "OG Roll: %f, OG Pitch: %f, OG Yaw: %f", roll, pitch, yaw);
      // I BELIEVE PITCH IS WHAT IS IMPORTANT

      while(true)
      { 
        // follow outside of obstacle adjusting angular speed to achieve this
        // safe distance from obstacle is 0.5
        // float setpoint = 0.5;
        // meas is the closest distance to the obstacle
        RCLCPP_INFO(this->get_logger(), "min distance to obstacle: %f", meas);
        RCLCPP_INFO(this->get_logger(), "obstacle is on right side: %d", right);

        

        break;

      }
    }

  private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
      robot = msg;
    }

    void topic_callback(const std_msgs::msg::String::SharedPtr msg)
    {
      current_action = msg->data;

      /*geometry_msgs::msg::TwistStamped ts_msg;
      ts_msg.twist.angular.x = 0;
      ts_msg.twist.angular.y = 0;
      ts_msg.twist.angular.z = 0;

      ts_msg.twist.linear.y = 0;
      ts_msg.twist.linear.z = 0;

      ts_msg.header.stamp = this->now();

      if(msg->data == "AVOID")
      {
        avoid_obs();
      }*/

      //publisher_1->publish(ts_msg);
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
      int range_len = msg->ranges.size();
      //float boundary = msg->range_min;
      int idx; //std::min_element(msg->ranges.begin(), msg->ranges.end());
      bool side = false;

      float lowest = msg->ranges[0];

      for(size_t i=0; i < (msg->ranges.size()); i++)
      {
        if(msg->ranges[i] < lowest)
        {
          lowest = msg->ranges[i];
          idx = i;
        }
      }

      if(idx > (range_len/2))
      {
        // assume turning right
        side = true;
      }

      if(current_action == "AVOID")
      {
        avoid_obs(lowest, side);
      }
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_1;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_2;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_3;

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_1;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_2;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControllerNode>());
  rclcpp::shutdown();
  return 0;
}