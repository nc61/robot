#include <robot/SerialCom.hpp>

using namespace LibSerial;

void SerialCom::init(std::string str)
{

    port.Open(str);
    if (!port.good())
    {
        std::cerr << "Error: Could not open serial port "
                  << str
                  << std::endl;
        exit(1);
    }
    
    port.SetBaudRate(SerialStreamBuf::BAUD_57600);
    if (!port.good())
    {
        std::cerr << "Error: Could not set baud rate for "
                  << str
                  << std::endl;
        exit(1);
    }
    
    port.SetCharSize(SerialStreamBuf::CHAR_SIZE_8);
    if (!port.good())
    {
        std::cerr << "Error: Could not set char size for "
                  << str
                  << std::endl;
        exit(1);
    }
    
    port.SetNumOfStopBits(1);
    if (!port.good())
    {
        std::cerr << "Error: Could not set num stop bits for "
                  << str
                  << std::endl;
        exit(1);
    }
   
    port.SetParity(SerialStreamBuf::PARITY_NONE);
    if (!port.good())
    { 
        std::cerr << "Error: Could not set parity for "
                  << str
                  << std::endl;
        exit(1);
    }
    
    port.SetFlowControl(SerialStreamBuf::FLOW_CONTROL_NONE);
    if (!port.good())
    {
        std::cerr << "Error: Could not set flow control for "
                  << str
                  << std::endl;
        exit(1);
    }

}

int SerialCom::serialWrite(char* data)
{
    port.write(data, sizeof(data));
    return port.good();
}

char* SerialCom::serialRead(int length)
{
    char* read = new char[length];
    port.read(read, length);
    return read;
}
