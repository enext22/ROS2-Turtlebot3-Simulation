#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
using std::placeholders::_1;

class ScanSubscriber : public rclcpp::Node
{
  public:
    std::string current_action;

    ScanSubscriber() : Node("scan_subscriber")
    {
      subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, std::bind(&ScanSubscriber::topic_callback, this, _1));

      subscription_2 = this->create_subscription<std_msgs::msg::String>(
      "/action", 10, std::bind(&ScanSubscriber::action_callback, this, _1));
      
      publisher_1 = this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);
      publisher_2 = this->create_publisher<std_msgs::msg::String>("/action", 10);
      ///timer_ = this->create_wall_timer(
      //500ms, std::bind(&DetectionPublisher::timer_callback, this));
    }

  private:
    void action_callback(const std_msgs::msg::String::SharedPtr act)
    {
        current_action = act->data;
    }

    void topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) const
    {
      RCLCPP_INFO(this->get_logger(), "Received LaserScan message with %zu ranges", msg->ranges.size());
    
      if(!msg->ranges.empty())
      {
        RCLCPP_INFO(this->get_logger(), "First range %f", msg->ranges[0]);
      }

      if(current_action == "AVOID")
      {
        return;
      }

      // Check all ranges and CAUSE ACTION for any which are within +3 tolerance of range_min
      for(size_t i=0; i < (msg->ranges.size()); i++)
      {
        float range = msg->ranges[i];
        if(range <= (msg->range_min))
        {
            RCLCPP_WARN(this->get_logger(), "TOO CLOSE TO TARGET XXX");
            geometry_msgs::msg::TwistStamped ts_msg;
            ts_msg.twist.angular.x = 0;
            ts_msg.twist.angular.y = 0;
            ts_msg.twist.angular.z = 0;

            ts_msg.twist.linear.x = 0;
            ts_msg.twist.linear.y = 0;
            ts_msg.twist.linear.z = 0;

            ts_msg.header.stamp = this->now();
            
            
            std_msgs::msg::String act_msg;
            act_msg.data = "AVOID";
            publisher_2->publish(act_msg);
            
            publisher_1->publish(ts_msg);
            break;
        }
        else
        {
            // check if there are objects nearby
        }
      }
    
    }
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_2;

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_1;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_2;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScanSubscriber>());
  rclcpp::shutdown();
  return 0;
}