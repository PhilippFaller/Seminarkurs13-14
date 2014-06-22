#ifndef DRONE_H
#define DRONE_H

#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/Twist.h"
#include "../../tum_ardrone/msg_gen/cpp/include/tum_ardrone/filter_state.h"

#include <thread>
#include <mutex>
#include <atomic>

#define PUBLISHING_RATE 50

class Drone{
private:
    //float x, y, z, yaw;
    //float goalX, goalY, goalZ, goalYaw;
    tum_ardrone::filter_state state;
    tum_ardrone::filter_state aimState;
    geometry_msgs::Twist currentMove;
    ros::NodeHandle nodeHandle;
    ros::Publisher takeOffPublisher;
    ros::Publisher landPublisher;
    ros::Publisher movePublisher;
    std::thread callbackThread;
    std::thread publishThread;
    std::thread navigationThread;
    std::mutex m;
    std::atomic_bool destruction;
    void updatePos(const tum_ardrone::filter_state &newState);
    void run();
    void callback();
    void navigate();
public:
    Drone();
    ~Drone();
    void takeoff();
    void land();
    void forward(float amount);
    void sideward(float amount);
    void up(float amount);
    tum_ardrone::filter_state getState();

};

#endif // DRONE_H
