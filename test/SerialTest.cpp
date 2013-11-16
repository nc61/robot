#include <string>
#include <stdlib.h>
#include "robot/SerialCom.hpp"

SerialCom test;
int main(int argc, char** argv)
{
    test.init("/dev/ttyUSB0"); 
    char cmd[] = {'b'};
    test.serialWrite(cmd);
    usleep(1000000);
    char ch;
    ch = test.readByte();
    printf("Response: %c", ch);
    printf("end");
    test.Close();
    
}


