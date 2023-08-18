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

      //Create timer object, will check if messages have been sent or not.
      timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&MasterSubscriber::timer_callback, this));
      last_msg_time_ = this->now();
      RCLCPP_INFO(this->get_logger(), "Succesffuly created timer with a period of 1 second.");

      //Create subscriber object that is subscribed to topic and bind it to node.
      subscription_ = this->create_subscription<geometry_msgs::msg::Vector3>(
      RPI_TOPIC, RPI_TOPIC_QUEUE, std::bind(&MasterSubscriber::topic_callback, this, _1));
      RCLCPP_INFO(this->get_logger(), "Successfully created subscriber to %s with a buffer queue of %i.", RPI_TOPIC.c_str(), RPI_TOPIC_QUEUE);
      RCLCPP_INFO(this->get_logger(), "Awaiting messages.");

      //TODO: Add various subscribers for the different components, figure out how to create modular code for timer_callback (not specific to message, 
      //maybe try using a template?)
      // Look into SDK/API/Library for the RS232 converter, motor, and strain gauges.
    }

  private:
    //Called every time a message is posted to a topic.
    void topic_callback(const geometry_msgs::msg::Vector3::SharedPtr msg)
    {
      last_msg_time_ = this->now();
    }
    
    //Function that checks if a message has been sent in the topic in the last 5 seconds or not.
    void timer_callback()
    {
      //If last message was greater than 5 seconds ago, print error to console.
      if((this->now() - last_msg_time_).seconds() > 5) {
        RCLCPP_ERROR(this->get_logger(),"Pi is not transmitting data. Check Connection");
      }
    }
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time last_msg_time_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  rclcpp::spin(std::make_shared<MasterSubscriber>());
  rclcpp::shutdown();
  return 0;
}