// Publishing new motion commands to gazebo
#include <memory>
#include <algorithm>

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
    ControllerNode() : Node("controller")
    {      
      subscription_1 = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&ControllerNode::odom_callback, this, _1));

      publisher_1 = this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);
    }

    nav_msgs::msg::Odometry::SharedPtr robot;
    std::string current_action;



  private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
      float x_pos = msg->pose.pose.position.x;
      float y_pos = msg->pose.pose.position.y;
      float z_pos = msg->pose.pose.position.z;
      float heading = msg->pose.pose.orientation.z; // yaw position

      printf("\nPosition: (%.2f, %.2f, %.2f) with Heading: %.2f", x_pos, y_pos, z_pos, heading);

      // Need to connect to obs_detection package/ node to decide on movement action
      // READ FORCE DATA
    }

    /*void path_callback(const nav_msgs::msg::Path msg)
    {
      float new_x = msg->poses.pose.position.x;
      float new_y = msg->poses.pose.position.y;
      float new_z = msg->poses.pose.position.z;

      float new_heading = msg->poses.pose.orientation.x;
    }*/

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_1;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_1;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControllerNode>());
  rclcpp::shutdown();
  return 0;
}

// Need to accept an INITIAL POSE, and a TARGET POSE
// Reconciling both with tthe global frame of reference from Gazebo

// After force calculation updates will be posted to cmd_vel during a 10Hz time step?