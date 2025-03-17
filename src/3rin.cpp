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
#include "omuni/omuni_utils.hpp"

using std::placeholders::_1;

class Omuni : public rclcpp::Node
{
private:
  void controller_callback(const sensor_msgs::msg::Joy & msg) const
  {
    if(msg.buttons[7]){
      omuni_setting_->publish(omuni::robomas_utils::to_velocity_mode(0));
      omuni_setting_->publish(omuni::robomas_utils::to_velocity_mode(1));
      omuni_setting_->publish(omuni::robomas_utils::to_velocity_mode(2));
    }
    if(msg.buttons[6]){
      omuni_setting_->publish(omuni::robomas_utils::to_disable_mode(0));
      omuni_setting_->publish(omuni::robomas_utils::to_disable_mode(1));
      omuni_setting_->publish(omuni::robomas_utils::to_disable_mode(2));
    }

    float V1,V2,V3 = 0;
    const float value = sqrt(3)/2;
    float rotation_value = 0.15;
    float Velocity = 300;

    if(msg.buttons[5]){
      Velocity = 400;
    } //加速する

    if(msg.axes[0] < 0 && msg.axes[1] < 0){
      V1 = Velocity*(-msg.axes[0]*msg.axes[0]-rotation_value*(1-msg.axes[5])*(1-msg.axes[5])-rotation_value*(1-msg.axes[2])*(1-msg.axes[2]));
      V2 = Velocity*(0.5*msg.axes[0]*msg.axes[0]+value*msg.axes[1]*msg.axes[1]+rotation_value*(1-msg.axes[5])*(1-msg.axes[5])-rotation_value*(1-msg.axes[2])*(1-msg.axes[2]));
      V3 = Velocity*(0.5*msg.axes[0]*msg.axes[0]-value*msg.axes[1]*msg.axes[1]+rotation_value*(1-msg.axes[5])*(1-msg.axes[5])-rotation_value*(1-msg.axes[2])*(1-msg.axes[2]));
    }
    else if(msg.axes[0] < 0){
      V1 = Velocity*(-msg.axes[0]*msg.axes[0]+rotation_value*(1-msg.axes[5])*(1-msg.axes[5])-rotation_value*(1-msg.axes[2])*(1-msg.axes[2]));
      V2 = Velocity*(0.5*msg.axes[0]*msg.axes[0]-value*msg.axes[1]*msg.axes[1]+rotation_value*(1-msg.axes[5])*(1-msg.axes[5])-rotation_value*(1-msg.axes[2])*(1-msg.axes[2]));
      V3 = Velocity*(0.5*msg.axes[0]*msg.axes[0]+value*msg.axes[1]*msg.axes[1]+rotation_value*(1-msg.axes[5])*(1-msg.axes[5])-rotation_value*(1-msg.axes[2])*(1-msg.axes[2]));
    }
    else if(msg.axes[1] < 0){
      V1 = Velocity*(msg.axes[0]*msg.axes[0]-rotation_value*(1-msg.axes[5])*(1-msg.axes[5])-rotation_value*(1-msg.axes[2])*(1-msg.axes[2]));
      V2 = Velocity*(-0.5*msg.axes[0]*msg.axes[0]+value*msg.axes[1]*msg.axes[1]+rotation_value*(1-msg.axes[5])*(1-msg.axes[5])-rotation_value*(1-msg.axes[2])*(1-msg.axes[2]));
      V3 = Velocity*(-0.5*msg.axes[0]*msg.axes[0]-value*msg.axes[1]*msg.axes[1]+rotation_value*(1-msg.axes[5])*(1-msg.axes[5])-rotation_value*(1-msg.axes[2])*(1-msg.axes[2]));
    } 
    else{
      V1 = Velocity*(msg.axes[0]*msg.axes[0]+rotation_value*(1-msg.axes[5])*(1-msg.axes[5])-rotation_value*(1-msg.axes[2])*(1-msg.axes[2]));
      V2 = Velocity*(-0.5*msg.axes[0]*msg.axes[0]-value*msg.axes[1]*msg.axes[1]+rotation_value*(1-msg.axes[5])*(1-msg.axes[5])-rotation_value*(1-msg.axes[2])*(1-msg.axes[2]));
      V3 = Velocity*(-0.5*msg.axes[0]*msg.axes[0]+value*msg.axes[1]*msg.axes[1]+rotation_value*(1-msg.axes[5])*(1-msg.axes[5])-rotation_value*(1-msg.axes[2])*(1-msg.axes[2]));
    }

    auto message1 = robomas_plugins::msg::RobomasTarget{};
    message1.target = V1;
    auto message2 = robomas_plugins::msg::RobomasTarget{};
    message2.target = V2;
    auto message3 = robomas_plugins::msg::RobomasTarget{};
    message3.target = V3; 

    omuni1_->publish(message1);
    omuni2_->publish(message2);
    omuni3_->publish(message3); //足回りの速度調整とロボマスに情報を送るための式。
  } 
  

public:
  Omuni()
  : Node("omuni")
  {
    this->controller_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&Omuni::controller_callback, this, std::placeholders::_1));
    this->omuni1_ = this->create_publisher<robomas_plugins::msg::RobomasTarget>("robomas_target1", 10);
    this->omuni2_ = this->create_publisher<robomas_plugins::msg::RobomasTarget>("robomas_target2", 10);
    this->omuni3_ = this->create_publisher<robomas_plugins::msg::RobomasTarget>("robomas_target3", 10);
    this->omuni_setting_  = this->create_publisher<robomas_plugins::msg::RobomasFrame>("robomas_frame", 10);
  }

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr controller_;
  rclcpp::Publisher<robomas_plugins::msg::RobomasTarget>::SharedPtr omuni1_;
  rclcpp::Publisher<robomas_plugins::msg::RobomasTarget>::SharedPtr omuni2_;
  rclcpp::Publisher<robomas_plugins::msg::RobomasTarget>::SharedPtr omuni3_;
  rclcpp::Publisher<robomas_plugins::msg::RobomasFrame>::SharedPtr omuni_setting_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Omuni>());
  rclcpp::shutdown();
  return 0;
}

