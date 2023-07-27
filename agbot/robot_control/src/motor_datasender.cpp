#include "motor_cmds.hpp" 

int main()
{
  auto robot_ = std::make_shared<RobotCommands>();

  // robot connection
  robot_->connection();

  while (true) {
    robot_->sending_data();
    std::this_thread::sleep_for(std::chrono::milliseconds(20)); // 100 ms pause for example
  }

  return 0;
}
