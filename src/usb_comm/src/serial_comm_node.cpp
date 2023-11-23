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

typedef struct{  
    double valueToChange;
}cubeDataWrite_t;

typedef struct{ // UART TX buffer 256 bytes
    uint32_t timeStamp;
    double dt;
    uint32_t measured_dt;
    double changedValue;
    bool calling_output_motors;
    bool in_output_func; //AKGL
    bool in_output_2_motors; //AKGL
    bool setting_values; //AKGL
    bool sending_2_motors; //AKGL 
    void printStruct()
    {
        std::cout << "Time stamp " << timeStamp << " \n";
        std::cout << "Dt " << dt << "\n";
        std::cout << "Measured dt " << measured_dt << "\n";
        std::cout << "Changed Value " << changedValue << "\n";
        std::cout << "calling_output_motors " << calling_output_motors << " \n";
        std::cout << "in_output_func " << in_output_func << "\n";
        std::cout << "in_output_2_motors " << in_output_2_motors << "\n";
        std::cout << "setting_values " << setting_values << "\n";
        std::cout << "sending_2_motors " << sending_2_motors << "\n";
    }
}cubeDataRead_t; 



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

        dataForCube_.valueToChange = 0.0;
        dataFromCube_ = { 0,   //timeStamp
                          0.0, //dt
                          0,   // measured dt
                          0.0, //changed value
                          false, // calling_output_motors
                          false, // in_output_func
                          false, // in_output_2_motors
                          false, // setting_values
                          false }; //sending_2_motors
    }

  private:
    void readUSB_callback()
    {
    //   publisher_->publish(message);
        read(fd_, (void*)&dataFromCube_, sizeof(cubeDataRead_t));  
        std::cout << "recv message  ********************* "<<std::endl;
        dataFromCube_.printStruct();

        // dataForCube_.valueToChange = 3.4;

        // std::cout << "sending \n";  
        // write (fd_, (void*)&dataForCube_, sizeof(cubeDataWrite_t)); 
        
    }
    rclcpp::TimerBase::SharedPtr timer_;
//    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    std::string portName_;
    int fd_; 
    cubeDataWrite_t dataForCube_;
    cubeDataRead_t dataFromCube_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<USBSerialComm>());
  rclcpp::shutdown();
  return 0;
}







