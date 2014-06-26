#include "ros/ros.h"
#include "../inc/drone.h"

using namespace ros;
int main(int argc, char** argv){
    init(argc, argv, "Seminarkurs");
    Drone d;
    d.takeoff();
    while(ros::ok());
    std::cout<<"Exit"<<std::endl;

}

