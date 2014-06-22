#include "../inc/drone.h"

void Drone::updatePos(const tum_ardrone::filter_state &newState)
{
    m.lock();
    state = newState;
    m.unlock();
}

void Drone::run(){
    ros::Rate r(PUBLISHING_RATE);
    while(!destruction && ros::ok()){
        m.lock();
        movePublisher.publish(currentMove);
        m.unlock();
        r.sleep();
    }
}

void Drone::callback()
{
    while(!destruction && ros::ok()) ros::spinOnce();
}

void Drone::navigate()
{
    while(!destruction && ros::ok()){
        m.lock();
        //x
        if(aimState.x < 0){
            if(state.x < aimState.x) currentMove.linear.x = 1;
            else if(state.x > aimState.x) currentMove.linear.x = -1;
            else currentMove.linear.x = 0;
        }
        else if(aimState.x > 0){
            if(state.x < aimState.x) currentMove.linear.x = -1;
            else if(state.x > aimState.x) currentMove.linear.x = 1;
            else currentMove.linear.x = 0;
        }
        else {
            if(state.x > 0) currentMove.linear.x = -1;
            else if(state.x < 0) currentMove.linear.x = 1;
            else currentMove.linear.x = 0;
        }
        //y
        if(aimState.y < 0){
            if(state.y < aimState.y) currentMove.linear.y = 1;
            else if(state.y > aimState.y) currentMove.linear.y = -1;
            else currentMove.linear.y = 0;
        }
        else if(aimState.y > 0){
            if(state.y < aimState.y) currentMove.linear.y = -1;
            else if(state.y > aimState.y) currentMove.linear.y = 1;
            else currentMove.linear.y = 0;
        }
        else {
            if(state.y > 0) currentMove.linear.y = -1;
            else if(state.y < 0) currentMove.linear.y = 1;
            else currentMove.linear.y = 0;
        }
        //z
        if(aimState.z < 0){
            if(state.z < aimState.z) currentMove.linear.z = 1;
            else if(state.z > aimState.z) currentMove.linear.z = -1;
            else currentMove.linear.z = 0;
        }
        else if(aimState.z > 0){
            if(state.z < aimState.z) currentMove.linear.z = -1;
            else if(state.z > aimState.z) currentMove.linear.z = 1;
            else currentMove.linear.z = 0;
        }
        else {
            if(state.z > 0) currentMove.linear.z = -1;
            else if(state.z < 0) currentMove.linear.z = 1;
            else currentMove.linear.z = 0;
        }

       m.unlock();
    }
}

Drone::Drone() : /*x(0), y(0), z(0), yaw(0), goalX(0), goalY(0), goalZ(0), goalYaw(0), */destruction(false)
{
    takeOffPublisher = nodeHandle.advertise<std_msgs::Empty>(nodeHandle.resolveName("ardrone/takeoff"),1, true);
    landPublisher = nodeHandle.advertise<std_msgs::Empty>(nodeHandle.resolveName("ardrone/land"), 1, true);
    movePublisher = nodeHandle.advertise<geometry_msgs::Twist>(nodeHandle.resolveName("cmd_vel"), 1, true);
    nodeHandle.subscribe(std::string("/ardrone/predictedPose"), 100, &Drone::updatePos, this);
    callbackThread = std::thread(&Drone::callback, this);
    publishThread = std::thread(&Drone::run, this);
    navigationThread = std::thread(&Drone::navigate, this);
}

Drone::~Drone()
{
    destruction = true;
    callbackThread.join();
    publishThread.join();
    navigationThread.join();
}

void Drone::takeoff()
{
    takeOffPublisher.publish(std_msgs::Empty());
}

void Drone::land()
{
    landPublisher.publish(std_msgs::Empty());

}

void Drone::forward(float amount)
{
    aimState.x = state.x + amount;
}

void Drone::sideward(float amount)
{
    aimState.y = state.y + amount;
}

void Drone::up(float amount)
{
    aimState.z = state.z + amount;
}

tum_ardrone::filter_state Drone::getState()
{
    return state;
}
