#include <boost/asio.hpp>
#include <iostream>

int main() {
  boost::asio::io_service io;
  boost::asio::serial_port serial(io, "/dev/ttyACM1");
  serial.set_option(boost::asio::serial_port_base::baud_rate(115200));

  char c;
  std::string finalValue;
  while (true) {
    boost::asio::read(serial, boost::asio::buffer(&c, 1));
    if (c == '\n') {
      break;
    }
    finalValue += c;
  }

  std::cout << "Final Value: " << finalValue << std::endl;

  return 0;
}
