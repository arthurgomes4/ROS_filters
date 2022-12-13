#include "ros/ros.h"
#include "std_msgs/Float64.h"

class EMAfilter
{
    float m_smoothingFactor;
    float m_previousValue;
    
    public:
    EMAfilter(){
        m_smoothingFactor = 0.5;
        m_previousValue = 0;
    }
    EMAfilter(float smoothingFactor){
        m_smoothingFactor = smoothingFactor;
        m_previousValue = 0;
    }

    float getSmoothingFactor(){
        return m_smoothingFactor;
    }

    void setSmoothingFactor(float newValue){
        m_smoothingFactor = newValue;
    }

    void resetFilter(float initialValue){
        m_previousValue = initialValue;
    }

    float filterValue(float currentValue){
        float filteredValue = currentValue*m_smoothingFactor + m_previousValue*(1-m_smoothingFactor);
        m_previousValue = filteredValue;
        return filteredValue;
    }
};

class ROS_EMAfilter
{
    EMAfilter m_filter;
    ros::Publisher m_publisher;
    ros::Subscriber m_subscriber;

    public:

    ROS_EMAfilter(ros::NodeHandle& nh, const char* inputTopic, const char* outputTopic, float smoothingFactor){
        m_filter.setSmoothingFactor(smoothingFactor);
        m_publisher = nh.advertise<std_msgs::Float64>(outputTopic, 10);
        m_subscriber = nh.subscribe(inputTopic, 10, &ROS_EMAfilter::callback, this);
    }

    void callback(std_msgs::Float64 message)
    {
        float rawValue = message.data;
        message.data = m_filter.filterValue(rawValue);
        m_publisher.publish(message);
        ROS_INFO("received: %f published: %f", rawValue, message.data);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ema_filter_node");
    ros::NodeHandle nh;
    ROS_EMAfilter joint1(nh, "joint1_command", "joint1_filtered", 0.3);
    ROS_INFO("ready for data");
    ros::spin();
    return 0;
}