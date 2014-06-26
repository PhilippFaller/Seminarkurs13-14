#include "../inc/drone.h"

void Drone::updatePos(const tum_ardrone::filter_state &newState)
{
    m.lock();
    state = newState;
    m.unlock();
}

void Drone::updateImg(const sensor_msgs::Image &newImg)
{
    m.lock();
    img = newImg;
    m.unlock();
}

void Drone::callbackLoop()
{
    while(!destruction && ros::ok()) ros::spinOnce();
}

Drone::Drone() : destruction(false)
{
    movePublisher  = nodeHandle.advertise<std_msgs::String>(nodeHandle.resolveName("/tum_ardrone/com"), 1, true);
    posSubscriber = nodeHandle.subscribe(nodeHandle.resolveName("/ardrone/predictedPose"), 1, &Drone::updatePos, this);
    imgSubscriber = nodeHandle.subscribe(nodeHandle.resolveName("ardrone/front/image_raw"), 1, &Drone::updateImg, this);
    callbackThread = std::thread(&Drone::callbackLoop, this);
    msg.data = "c start";
    movePublisher.publish(msg);
}

Drone::~Drone()
{
    destruction = true;
    msg.data = "c stop";
    movePublisher.publish(msg);
    callbackThread.join();
}

void Drone::takeoff()
{
    msg.data = "c autoInit 500 800 4000 0.5";
    movePublisher.publish(msg);
}

void Drone::land()
{
    msg.data = "c land";
    movePublisher.publish(msg);
}

void Drone::forward(float amount)
{
    msg.data = std::string("c moveByRel ") + std::to_string(amount) + std::string(" 0 0 0");
    movePublisher.publish(msg);
}

void Drone::right(float amount)
{
    msg.data = std::string("c moveByRel 0") + std::to_string(amount) + std::string(" 0 0");
    movePublisher.publish(msg);
}

void Drone::up(float amount)
{
    msg.data = std::string("c moveByRel 0 0") + std::to_string(amount) + std::string(" 0");
    movePublisher.publish(msg);
}

tum_ardrone::filter_state Drone::getState()
{
    return state;
}

cv::Mat Drone::getCVImageFront()
{
    cv_bridge::CvImagePtr cvPointer;
    try{
        cvPointer = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return cv::Mat(1, 1, 0);
    }
    return cvPointer->image;
}
