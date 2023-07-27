#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "motor_cmds.hpp" 

class MotorController : public rclcpp::Node
{
public:
  MotorController()
    : Node("motor_controller")
  {
    robot_ = std::make_shared<RobotCommands>();

    subscription_ = this->create_subscription<std_msgs::msg::Int16MultiArray>(
      "motor_command",
      10,
      std::bind(&MotorController::listener_callback, this, std::placeholders::_1));

    // robot connection
    robot_->connection();

    // init command
    robot_->set_left_speed(255);
    robot_->set_right_speed(255);
    robot_->set_direction_ctrl(0);
  }

private:
  void listener_callback(const std_msgs::msg::Int16MultiArray::SharedPtr msg)
  {
    int motor_left = msg->data[0];
    int motor_right = msg->data[1];

    // robot_->set_left_speed(motor_left);
    // robot_->set_right_speed(motor_right);

    // sending motor command
    robot_->sending_data();

  }

  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr subscription_;
  std::shared_ptr<RobotCommands> robot_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotorController>());
  rclcpp::shutdown();
  return 0;
}
