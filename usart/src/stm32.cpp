#include "ros/ros.h"
#include "serial/serial.h"
#include <iostream>

#define port "/dev/ttyUSB0"
#define baud 119200

using namespace std;

int main(int argc, char* argv[]){
    ros::init(argc,argv,"stm32");
    serial::Serial my_serial(port, baud, serial::Timeout::simpleTimeout(1000));
    cout << "Is the serial port open?";
    if(my_serial.isOpen())
        cout << " Yes." << endl;
    else
        cout << " No." << endl;

    return 0;
}