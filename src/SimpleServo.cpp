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
    
    maestro.SetBaudRate(SerialStreamBuf::BAUD_115200);
    maestro.SetCharSize(SerialStreamBuf::CHAR_SIZE_8);
    maestro.SetNumOfStopBits(1);
    maestro.SetParity(SerialStreamBuf::PARITY_NONE);
    maestro.SetFlowControl(SerialStreamBuf::FLOW_CONTROL_NONE);

    char cmd[] = {0xAA, 0x0C, 0x04, 0x04, 0x70, 0x2E};
    maestro.write(cmd, sizeof(cmd));
    
    maestro.Close();
}
