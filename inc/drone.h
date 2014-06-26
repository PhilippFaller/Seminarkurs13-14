#ifndef DRONE_H
#define DRONE_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "../../tum_ardrone/msg_gen/cpp/include/tum_ardrone/filter_state.h"

#include "cv_bridge/cv_bridge.h"

#include <thread>
#include <mutex>
#include <atomic>

#define PUBLISHING_RATE 50

class Drone{
private:
    tum_ardrone::filter_state state;
    sensor_msgs::Image img;
    ros::NodeHandle nodeHandle;
    ros::Publisher movePublisher;
    ros::Subscriber posSubscriber;
    ros::Subscriber imgSubscriber;
    std::thread callbackThread;
    std::mutex m;
    std::atomic_bool destruction;
    void updatePos(const tum_ardrone::filter_state &newState);
    void updateImg(const sensor_msgs::Image &newImg);
    void callbackLoop();
public:
    Drone();
    ~Drone();
    void takeoff();
    void land();
    void forward(float amount);
    void right(float amount);
    void up(float amount);
    tum_ardrone::filter_state getState();
    cv::Mat getCVImageFront();

};

#endif // DRONE_H
