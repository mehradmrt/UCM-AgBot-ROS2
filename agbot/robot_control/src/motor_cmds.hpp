#include <array>
#include <memory>
#include <boost/asio.hpp>
#include <iostream> 

#ifndef ROBOT_CONTROL_HPP
#define ROBOT_CONTROL_HPP

class RobotCommands
{
public:
  RobotCommands()
    : socket_(io_context_),
      server_("192.168.0.7"),
      port_(20001),
      lift_speed_(0),
      lift_ctrl_(0),
      left_motor_speed_(0),
      right_motor_speed_(0),
      motor_rotation_ctrl_(0),
      data_({0x08, 0x00, 0x00, 0x00, 0x12, 0x00, 0x00, 0x00, 0x00, 
             0x00, 0x00, 
             0x00, 0x38})

  // {
  //     data_[6] = lift_speed_;
  //     data_[7] = lift_ctrl_;
  //     data_[9] = left_motor_speed_;
  //     data_[10] = right_motor_speed_;
  //     data_[11] = motor_rotation_ctrl_;
  // }

  {}
  void connection()
  {
    socket_.connect(boost::asio::ip::tcp::endpoint(boost::asio::ip::address::from_string(server_), port_));
    std::cout << "Connection Done\n";
  }

  void set_lift_speed(int speed)
  {
    data_[6] = speed;
  }

  void set_lift_ctrl(int ctrl)
  {
    data_[7] = ctrl;
  }

  void set_left_speed(int speed)
  {
    data_[9] = speed;
  }

  void set_right_speed(int speed)
  {
    data_[10] = speed;
  }

  void set_direction_ctrl(int state)
  {
    data_[11] = state;
  }

  void sending_data()
  {
    boost::asio::write(socket_, boost::asio::buffer(data_));
    // std::this_thread::sleep_for(std::chrono::milliseconds(30));

    std::cout << "Sending data: ";
    for(const auto &value : data_)
    {
        std::cout << static_cast<int>(value) << ' ';
    }
    std::cout << std::endl;
  }

private:
  boost::asio::io_context io_context_;
  boost::asio::ip::tcp::socket socket_;
  std::string server_;
  int port_;

  int lift_speed_;
  int lift_ctrl_;
  int left_motor_speed_;
  int right_motor_speed_;
  int motor_rotation_ctrl_;

  std::array<unsigned char, 13> data_;
};

#endif
