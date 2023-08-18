#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"
using std::placeholders::_1;

const std::string RPI_TOPIC = "rpi_imu_ypr_topic";
const int RPI_TOPIC_QUEUE = 10;

class MasterSubscriber : public rclcpp::Node
{
  public:
    MasterSubscriber()
    : Node("master_subscriber")
    {
      RCLCPP_INFO(this->get_logger(), "Initialized node 'master_subscriber'.");
      subscription_ = this->create_subscription<geometry_msgs::msg::Vector3>(
      RPI_TOPIC, RPI_TOPIC_QUEUE, std::bind(&MasterSubscriber::topic_callback, this, _1));
      RCLCPP_INFO(this->get_logger(), "Successfully created subscriber to %s with a buffer queue of %i.\n Awaiting messages.", RPI_TOPIC.c_str(), RPI_TOPIC_QUEUE);
    }

  private:
    void topic_callback(const geometry_msgs::msg::Vector3::SharedPtr msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: 'x: %f, y: %f, z: %f'", msg->x, msg->y, msg->z);
    }
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  rclcpp::spin(std::make_shared<MasterSubscriber>());
  rclcpp::shutdown();
  return 0;
}