#ifndef _SERIAL_COM_
#define _SERIAL_COM_

#include <SerialStream.h>
#include <string>
#include <iostream>
#include <stdlib.h>
#include "ros/ros.h"

using namespace LibSerial;

class SerialCom
{
        SerialStream port;
    public:
        void init(std::string str);
        int serialWrite(char* data);
        char* serialRead(int length);
};

#endif
