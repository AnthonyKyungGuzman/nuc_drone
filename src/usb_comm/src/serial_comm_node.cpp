#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <errno.h>
#include <fcntl.h> 
#include <string.h>
#include <termios.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

int set_interface_attribs (int fd, int speed, int parity)
{
        struct termios tty;
        if (tcgetattr (fd, &tty) != 0)
        {
//                error_message ("error %d from tcgetattr", errno);
		std::cout << "Error " << errno << "from tcgetattr\n";
                return -1;
        }

        cfsetospeed (&tty, speed);
        cfsetispeed (&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
        // disable IGNBRK for mismatched speed tests; otherwise receive break
        // as \000 chars
        tty.c_iflag &= ~IGNBRK;         // disable break processing
        tty.c_lflag = 0;                // no signaling chars, no echo,
                                        // no canonical processing
        tty.c_oflag = 0;                // no remapping, no delays
        tty.c_cc[VMIN]  = 0;            // read doesn't block
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

        tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                        // enable reading
        tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
        tty.c_cflag |= parity;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
        {
//                error_message ("error %d from tcsetattr", errno);
		std::cout << "Error " << errno << " from tcsetattr\n";                                
                return -1;
        }
        return 0;
}


void set_blocking (int fd, int should_block)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
//                error_message ("error %d from tggetattr", errno);
		std::cout << "Error " << errno << " from tggetattr\n";                
                return;
        }

        tty.c_cc[VMIN]  = should_block ? 1 : 0;
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
	{
               // error_message ("error %d setting term attributes", errno);
		std::cout << "Error " << errno << " setting term attributes\n";
	}
}



/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class USBSerialComm : public rclcpp::Node
{
  public:
    USBSerialComm()
    : Node("USB_serial_comm")
    {
    //   publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
        this->declare_parameter("serial_port", "/dev/ttyUSB1");
        portName_ = this->get_parameter("serial_port").as_string(); 
        std::cout << "Serial port " << portName_ <<std::endl;
        timer_ = this->create_wall_timer(
            1000ms, std::bind(&USBSerialComm::readUSB_callback, this));
        fd_= open (portName_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (fd_ < 0)
        {
            //error_message ("error %d opening %s: %s", errno, portName_, strerror (errno));
           std::cout << "Error "<< errno << " opening " << portName_ << " : " << strerror(errno) <<std::endl;
        }
        set_interface_attribs (fd_, B115200, 0);  // set speed to 115,200 bps, 8n1 (no parity)
        set_blocking (fd_, 1);                    // set blocking
        std::cout << "Finish init \n";
    }

  private:
    void readUSB_callback()
    {
    //   publisher_->publish(message);
        std::cout << "sending \n";

        // write (fd_, "hello!\n", 7);           // send 7 character greeting
        // usleep ((7 + 25) * 100);             // sleep enough to transmit the 7 plus
                                            // receive 25:  approx 100 uS per char transmit

        char buf [10];
        int n = read(fd, buf, 4);  // read up to 100 characters if ready to read
        std::string message(buf);
        std::cout << "recv message " << message <<std::endl;
    }
    rclcpp::TimerBase::SharedPtr timer_;
//    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    std::string portName_;
    int fd_; 

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<USBSerialComm>());
  rclcpp::shutdown();
  return 0;
}







