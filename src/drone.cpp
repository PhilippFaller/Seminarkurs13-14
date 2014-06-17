#include "../inc/drone.h"

Drone::Drone() : x(0), y(0), z(0)
{
    takeOffPublisher = nodeHandle.advertise<std_msgs::Empty>("ardrone/takeoff", 1);
    landPublisher = nodeHandle.advertise<std_msgs::Empty>("/ardrone/land", 1);
}

void Drone::takeoff()
{
    takeOffPublisher.publish(std_msgs::Empty());
}

void Drone::land()
{
    landPublisher.publish(std_msgs::Empty());

}
