#include <memory>
#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "motor_cmds.hpp"


class MotorController : public rclcpp::Node
{
public:
  MotorController()
    : Node("motor_controller"), run_data_sending(true)  // Initialize run_data_sending
  {
    robot_ = std::make_shared<RobotCommands>();

    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel",
      10,
      std::bind(&MotorController::listener_callback, this, std::placeholders::_1));

    // robot connection
    robot_->connection();

    // init command
    set_speed_and_direction(0.0, 0.0);

    // Start data sending thread
    data_sending_thread_ = std::thread(&MotorController::send_data_continuously, this);
  }

  ~MotorController() {
    // Signal data sending thread to stop and wait for it to finish
    run_data_sending = false;
    if (data_sending_thread_.joinable()) {
      data_sending_thread_.join();
    }
  }

private:
  int map_to_motor_range(double normalized_speed)
  { 
    int max_speed_value = 200;
    int min_speed_value = 50;
    double mapped_speed = ( std::abs(normalized_speed) * max_speed_value ) + min_speed_value; // map [0, 1] -> [50, 200]
    if (mapped_speed < min_speed_value + 5){
        mapped_speed = 0;
    }
    return static_cast<int>(mapped_speed);
  }

  void set_speed_and_direction(double left_speed, double right_speed)
  {
    int direction = 0;
    if (left_speed < 0 && right_speed < 0) {
      direction = 3; // both are negative
    } else if (left_speed < 0 && right_speed > 0) {
      direction = 2; // left_motor is - and right is +
    } else if (right_speed < 0 && left_speed > 0) {
      direction = 1; // right_motor is - and left is +
    }

    robot_->set_direction_ctrl(direction);
    robot_->set_left_speed(map_to_motor_range(left_speed));
    robot_->set_right_speed(map_to_motor_range(right_speed));
  }

  void listener_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    double linear_vel = msg->linear.x;
    double angular_vel = msg->angular.z;

    // Parameters specific to your robot
    double max_speed = 2.0;  // maximum speed in m/s
    double wheel_distance = 0.8;  // distance between wheels in meters

    // Convert Twist commands to differential drive motor commands
    double motor_left_normalized = (linear_vel - angular_vel * wheel_distance / 2.0) / max_speed;
    double motor_right_normalized = (linear_vel + angular_vel * wheel_distance / 2.0) / max_speed;

    RCLCPP_INFO(this->get_logger(), "Receiving data: left: %f, right: %f", motor_left_normalized, motor_right_normalized);

    // Change motor command
    set_speed_and_direction(motor_left_normalized, motor_right_normalized);
  }

  void send_data_continuously() {
    while (run_data_sending) {
      robot_->sending_data();
    //   std::this_thread::sleep_for(std::chrono::milliseconds(20));  // Sleep to control rate
    }
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  std::shared_ptr<RobotCommands> robot_;
  bool run_data_sending;  // Flag to control data sending thread
  std::thread data_sending_thread_;  // Data sending thread
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotorController>());
  rclcpp::shutdown();
  return 0;
}
