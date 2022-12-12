#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include<string.h>

class EMA_filter
{
    float m_smoothingFactor;
    float m_previousValue = 0;
    
    public:
    EMA_filter(float smoothingFactor = 0.5){
        m_smoothingFactor = smoothingFactor;
        }

    float filterValue(float currentValue){
        float filteredValue = currentValue*m_smoothingFactor + m_previousValue*(1-m_smoothingFactor);
        m_previousValue = filteredValue;
        return filteredValue;
    }
};

float temp;
ros::Publisher pub;
EMA_filter filter1(0.5);

void callback(std_msgs::Float64 message){
    message.data = filter1.filterValue(message.data);
    pub.publish(message);
    ROS_INFO("published: %f", message.data);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ema_filter_node");
    ros::NodeHandle nh;

    const char *inputTopic = "/joint1_command", *outputTopic="/joint1_filtered";

    pub = nh.advertise<std_msgs::Float64>(outputTopic, 10);
    ros::Subscriber sub = nh.subscribe(inputTopic, 10, callback);
    
    ROS_INFO("ready");
    ros::spin();

    return 0;
}