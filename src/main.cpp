#include "ros/ros.h"
#include "../inc/drone.h"

using namespace ros;
int main(int argc, char** argv){
    init(argc, argv, "Seminarkurs");
    Drone d;
    d.takeoff();

//    ros::NodeHandle n;
//    ros::Publisher p = n.advertise<std_msgs::String>("/tum_ardrone/com", 1, true);
//    std_msgs::String msg;
//    msg.data = "c start";
//    p.publish(msg);
    while(ros::ok());
}

