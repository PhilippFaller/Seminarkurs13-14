#include "ros/ros.h"
#include "../inc/drone.h"

using namespace ros;
int main(int argc, char** argv){
    init(argc, argv, "Seminarkurs");
    Drone d;
    d.takeoff();
    ros::Rate r(1);
    r.sleep();
    r.sleep();
    r.sleep();
    d.land();

}

