#include "ros/ros.h"
#include "../inc/drone.h"
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"


    tum_ardrone::filter_state state;
    cv::Mat *img = 0;
    std_msgs::String msg;
    ros::NodeHandle *nodeHandle = 0;
    ros::Publisher movePublisher;
    ros::Subscriber posSubscriber;
    ros::Subscriber imgSubscriber;
    std::thread callbackThread;
    std::mutex m;
    std::atomic_bool destruction;

    cv::CascadeClassifier right_arrow;
    std::string classifierName = "/home/philipp/fuerte_workspace/Seminarkurs/classifiers/haarcascade_frontalface_alt.xml";

    void updatePos(const tum_ardrone::filter_state &newState)
{
    m.lock();
    state = newState;
    m.unlock();
}

    void updateImg(const sensor_msgs::ImageConstPtr &newImg)
{
    m.lock();
    cv_bridge::CvImagePtr cvPointer;
    try{
        cvPointer = cv_bridge::toCvCopy(newImg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    delete img;
    img = new cv::Mat(cvPointer->image);
    m.unlock();
}
    void callbackLoop()
{
    while(!destruction && ros::ok()) ros::spinOnce();
}


    void takeoff()
{
    msg.data = "c autoInit 500 800 4000 0.5";
    movePublisher.publish(msg);
    ros::Rate second(1);
    second.sleep();
}

    void land()
{
    msg.data = "c land";
    movePublisher.publish(msg);
    ros::Rate second(1);
    second.sleep();
}

    void forward(float amount)
{
    msg.data = std::string("c moveByRel ") + std::to_string(amount) + std::string(" 0 0 0");
    movePublisher.publish(msg);
}

    void right(float amount)
{
    msg.data = std::string("c moveByRel 0") + std::to_string(amount) + std::string(" 0 0");
    movePublisher.publish(msg);
}

    void up(float amount)
{
    msg.data = std::string("c moveByRel 0 0") + std::to_string(amount) + std::string(" 0");
    movePublisher.publish(msg);
}

    tum_ardrone::filter_state getState()
{
    return state;
}

    cv::Mat getCVImageFront()
{
    return *img;
}


void init(){
    if(!right_arrow.load(classifierName)) std::cerr<<"Failed to load Classifier."<<std::endl;
    destruction = false;
    nodeHandle = new ros::NodeHandle();
    movePublisher  = nodeHandle->advertise<std_msgs::String>(nodeHandle->resolveName("/tum_ardrone/com"), 50, true);
    posSubscriber = nodeHandle->subscribe(nodeHandle->resolveName("/ardrone/predictedPose"), 1, updatePos);
    imgSubscriber = nodeHandle->subscribe(nodeHandle->resolveName("ardrone/front/image_raw"), 1, updateImg);
    callbackThread = std::thread(callbackLoop);
    msg.data = "c start";
    movePublisher.publish(msg);
    ros::Rate second(1);
    second.sleep();
}

void finish(){
    destruction = true;
    msg.data = "c land";
    movePublisher.publish(msg);
    ros::Rate second(1);
    second.sleep();
    msg.data = "c stop";
    movePublisher.publish(msg);
    second.sleep();
    callbackThread.join();
    delete nodeHandle;
}

bool detectRightArrow(cv::Mat frame){
    std::vector<cv::Rect> arrows;
    cv::Mat frame_gray;
    cv::cvtColor(frame, frame_gray, CV_BGR2GRAY);
    equalizeHist(frame_gray, frame_gray);
    right_arrow.detectMultiScale( frame_gray, arrows, 1.1, 2, CV_HAAR_SCALE_IMAGE, cv::Size(30, 30));
    for(unsigned int i = 0; i < arrows.size(); i++)
    {
    cv::Point center(arrows[i].x + arrows[i].width * 0.5, arrows[i].y + arrows[i].height * 0.5);
    cv::ellipse(frame, center, cv::Size(arrows[i].width * 0.5, arrows[i].height * 0.5), 0, 0, 360, cv::Scalar(255, 0, 255), 4, 8, 0);
    }
    cv::namedWindow("Seminarkurs", cv::WINDOW_AUTOSIZE);
    cv::imshow("Seminarkurs", frame);
    cv::waitKey(1);
    if(arrows.size() != 0) return true;
    return false;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "Seminarkurs");
    ::init();
    takeoff();
    ros::Rate freq(10);    
    while(ros::ok()){
    if(img) if(detectRightArrow(getCVImageFront())) right(0.1);
        freq.sleep();
    }
    finish();
}

