#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"
using std::placeholders::_1;

class MasterSubscriber : public rclcpp::Node
{
  public:
    MasterSubscriber()
    : Node("master_subscriber")
    {
      subscription_ = this->create_subscription<geometry_msgs::msg::Vector3>(
      "rpi_imu_ypr_topic", 10, std::bind(&MasterSubscriber::topic_callback, this, _1));
    }

  private:
    void topic_callback(const std_msgs::msg::String & msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MasterSubscriber>());
  rclcpp::shutdown();
  return 0;
}