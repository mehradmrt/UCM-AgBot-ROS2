#include <memory>
#include <chrono>
#include <thread>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "motor_cmds.hpp"

class MotorController : public rclcpp::Node
{
public:
    MotorController()
        : Node("motor_controller")
    {
        robot_ = std::make_shared<RobotCommands>();

        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel",
            10,
            std::bind(&MotorController::listener_callback, this, std::placeholders::_1));

        robot_->connection();  // robot connection
        set_speed_and_direction(0.0, 0.0);  // init command
    }

private:
    void listener_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        double linear_vel = msg->linear.x;
        double angular_vel = msg->angular.z;

        // double max_speed = 0.175;
        double wheel_distance = 0.8;

        double motor_left_normalized = (linear_vel - angular_vel * wheel_distance / 2.0);
        double motor_right_normalized = (linear_vel + angular_vel * wheel_distance / 2.0);

        start_velocity_timer(motor_left_normalized, motor_right_normalized);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    void start_velocity_timer(double left_speed, double right_speed)
    {
        auto period = std::chrono::milliseconds(50); // 20Hz update
        timer_ = this->create_wall_timer(
            period,
            [this, left_speed, right_speed]() {
                set_speed_and_direction(left_speed, right_speed);
                robot_->sending_data();
            });
    }


    void check_status_and_send(double left_speed, double right_speed)
    {
        int current_rotation, next_rotation;

        {
            std::lock_guard<std::mutex> lock(robot_mutex_);
            current_rotation = robot_->rotation_check();
        }
        
        next_rotation = direction_corrector(left_speed, right_speed);

        //loop to send data 3 times with 20ms intervals
        while (current_rotation != next_rotation)
        {
            {
                std::lock_guard<std::mutex> lock(robot_mutex_);
                //robot_->set_neutral();
                for (int i = 0; i < 50; i++)
                {

                    while (!robot_->is_connected()) 
                        {
                          std::cout << "Connection lost. Attempting to reconnect...\n";
                          robot_->connection();  
                        }
                    robot_->sending_data();

                    // Sleep for 20ms to maintain 50Hz frequency
                    std::this_thread::sleep_for(std::chrono::milliseconds(20));
                }
            }
            current_rotation = next_rotation;
        }

        set_speed_and_direction(left_speed, right_speed);
    }

    void set_speed_and_direction(double left_speed, double right_speed)
    {
        std::lock_guard<std::mutex> lock(robot_mutex_);
        robot_->set_direction_ctrl(direction_corrector(left_speed, right_speed));
        robot_->set_left_speed(map_to_motor_range(left_speed));
        robot_->set_right_speed(map_to_motor_range(right_speed));
        robot_->sending_data();
    }

    int map_to_motor_range(double _speed)
    {
        int max_speed_value = 225;
        int min_speed_value = 73;
        double mapped_speed = (std::abs(_speed) * (max_speed_value - min_speed_value)) + min_speed_value; // map [0, 1] -> [50, 200]
        if (mapped_speed < min_speed_value + 1)
        {
            mapped_speed = 0.0;
        }
        else if (mapped_speed > max_speed_value)
        {
            mapped_speed = max_speed_value;
        }
        return static_cast<int>(std::round(mapped_speed));
    }

    int direction_corrector(double left_speed, double right_speed)
    {
        int direction;
        if (right_speed > 0 && left_speed > 0)
        {
            direction = 0; // forward
        }
        else if (left_speed > 0 && right_speed < 0)
        {
            direction = 1; // cw
        }
        else if (left_speed < 0 && right_speed > 0)
        {
            direction = 2; // ccw           
        }
        else if (left_speed < 0 && right_speed < 0)
        {
            direction = 3; // reverse
        }

        return direction;
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    std::shared_ptr<RobotCommands> robot_;
    std::mutex robot_mutex_; // Mutex for thread safety
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorController>());
    rclcpp::shutdown();
    return 0;
}