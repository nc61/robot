#include <SerialStream.h>
#include <string>
#include <iostream>
#include <stdlib.h>
using namespace LibSerial;

int main(int argc, char** argv){
     
    SerialStream maestro;
    maestro.Open("/dev/ttyACM0");

    if (!maestro.good())
    {
        std::cerr << "[" << __FILE__ << ":" << __LINE__ << "] "
                  << "Error: Could not open serial port."
                  << std::endl;
        exit(1);
    }
    
    maestro.SetBaudRate(SerialStreamBuf::BAUD_57600);
    if (!maestro.good())
    {
        std::cerr << "[" << __FILE__ << ":" << __LINE__ << "] "
                  << "Error: Could not set baud rate."
                  << std::endl;
        exit(1);
    }
    maestro.SetCharSize(SerialStreamBuf::CHAR_SIZE_8);
    if (!maestro.good())
    {
        std::cerr << "[" << __FILE__ << ":" << __LINE__ << "] "
                  << "Error: Could not set char size."
                  << std::endl;
        exit(1);
    }
    maestro.SetNumOfStopBits(1);
    if (!maestro.good())
    {
        std::cerr << "[" << __FILE__ << ":" << __LINE__ << "] "
                  << "Error: Could not set num stop bits."
                  << std::endl;
        exit(1);
    }
    maestro.SetParity(SerialStreamBuf::PARITY_NONE);
    if (!maestro.good())
    {
        std::cerr << "[" << __FILE__ << ":" << __LINE__ << "] "
                  << "Error: Could not set parity."
                  << std::endl;
        exit(1);
    }
    maestro.SetFlowControl(SerialStreamBuf::FLOW_CONTROL_NONE);
    if (!maestro.good())
    {
        std::cerr << "[" << __FILE__ << ":" << __LINE__ << "] "
                  << "Error: Could not set flow control."
                  << std::endl;
        exit(1);
    }

    while(1){
        char cmd[] = {0x84, 0x00, 0x00, 0x3E};
        maestro.write(cmd, sizeof(cmd));
        usleep(1000000);
    }
    maestro.Close();
}
