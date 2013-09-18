#include <SerialStream.h>
#include <string>
#include <iostream>
#include <stdlib.h>

using namespace LibSerial;

int readPin(unsigned int channel);
SerialStream maestro;

int main(){
    
    int analogValue;

    maestro.Open("/dev/ttyACM0");

    if (!maestro.good())
    {
        std::cerr << "[" << __FILE__ << ":" << __LINE__ << "] "
                  << "Error: Could not open serial port."
                  << std::endl ;
        exit(1) ;
    }
    
    maestro.SetBaudRate(SerialStreamBuf::BAUD_115200);
    maestro.SetCharSize(SerialStreamBuf::CHAR_SIZE_8);
    maestro.SetNumOfStopBits(1);
    maestro.SetParity(SerialStreamBuf::PARITY_NONE);
    maestro.SetFlowControl(SerialStreamBuf::FLOW_CONTROL_HARD);

    std::cout << "Press x to exit";
    while(1)
    {
        analogValue = readPin(0);
        std::cout << "Received data: " << analogValue << std::endl;
        usleep(100000);
    }

    maestro.Close();
}

int readPin(unsigned int channel)
{

    char command[] = {0x90, channel};
    maestro.write(command, sizeof(command));
    
    char response[2];
    maestro.read(response, sizeof(response));
    
    return response[0] + 256*response[1];
}


