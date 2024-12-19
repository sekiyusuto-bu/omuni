#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>
#include <sensor_msgs/msg/joy.hpp>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "robomas_plugins/msg/robomas_target.hpp"
#include "robomas_plugins/msg/robomas_frame.hpp"

using std::placeholders::_1;

class Omuni : public rclcpp::Node
{
private:
  void controller_callback(const sensor_msgs::msg::Joy & msg) const
  {
    float V1,V2,V3 = 0;
    float Velocity = 200;
    float value = sqrt(3)/2;

    V1 = Velocity*(msg.axes[2]-msg.buttons[4]+msg.buttons[5]);
    V2 = Velocity*(-0.5*msg.axes[2]+value*msg.axes[3]-msg.buttons[4]+msg.buttons[5]);
    V3 = Velocity*(-0.5*msg.axes[2]-value*msg.axes[3]-msg.buttons[4]+msg.buttons[5]);

    auto message1 = robomas_plugins::msg::RobomasTarget{};
    message1.target = V1;
    auto message2 = robomas_plugins::msg::RobomasTarget{};
    message2.target = V2;
    auto message3 = robomas_plugins::msg::RobomasTarget{};
    message3.target = V3;

    omuni1_->publish(message1);
    omuni2_->publish(message2);
    omuni3_->publish(message3);
  } 
  

public:
  Omuni()
  : Node("omuni")
  {
    this->controller_ = this->create_subscription<sensor_msgs::msg::Joy>("Joy", 10, std::bind(&Omuni::controller_callback, this, std::placeholders::_1));
    this->omuni1_ = this->create_publisher<robomas_plugins::msg::RobomasTarget>("robomas_target1", 10);
    this->omuni2_ = this->create_publisher<robomas_plugins::msg::RobomasTarget>("robomas_target2", 10);
    this->omuni3_ = this->create_publisher<robomas_plugins::msg::RobomasTarget>("robomas_target3", 10);
  }

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr controller_;
  rclcpp::Publisher<robomas_plugins::msg::RobomasTarget>::SharedPtr omuni1_;
  rclcpp::Publisher<robomas_plugins::msg::RobomasTarget>::SharedPtr omuni2_;
  rclcpp::Publisher<robomas_plugins::msg::RobomasTarget>::SharedPtr omuni3_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Omuni>());
  rclcpp::shutdown();
  return 0;
}

