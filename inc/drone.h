#ifndef DRONE_H
#define DRONE_H

#include "ros/ros.h"
#include "std_msgs/Empty.h"

using namespace ros;

class Drone{
private:
    int x;
    int y;
    int z;
    NodeHandle nodeHandle;
    Publisher takeOffPublisher;
    Publisher landPublisher;
public:
    Drone();
    void takeoff();
    void land();


};

#endif // DRONE_H
