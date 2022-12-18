#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "../include/ema_filter.h"

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
        ROS_INFO("raw:/%s | filtered:/%s | smoothing:%f",inputTopic, outputTopic, smoothingFactor);
    }

    void callback(std_msgs::Float64 message)
    {
        float rawValue = message.data;
        message.data = m_filter.filterValue(rawValue);
        m_publisher.publish(message);
        // ROS_INFO("received: %f published: %f", rawValue, message.data);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ema_filter_node");
    ros::NodeHandle nh;

    if(!ros::param::has("~smoothingFactors")){
        ROS_ERROR("private param <smoothingFactors> not found\nThis is a list of floats\neach topic will have its own smoothingfactor");
        ros::shutdown();
        return 0;
    }
    if(!ros::param::has("~inputTopics")){
        ROS_ERROR("private param <inputTopics> not found\nThis is a list of strings");
        ros::shutdown();
        return 0;
    }
    if(!ros::param::has("~outputTopics")){
        ROS_ERROR("private param <outputTopics> not found\n this is a list of strings");
        ros::shutdown();
        return 0;
    }

    std::vector<std::string> inputTopics;
    std::vector<std::string> outputTopics;
    std::vector<float> smoothingFactors;

    ros::param::get("~smoothingFactors", smoothingFactors);
    ros::param::get("~inputTopics", inputTopics);
    ros::param::get("~outputTopics", outputTopics);

    std::vector<ROS_EMAfilter*> filters;

    for(int i=0; i<inputTopics.size(); i++)
    {
        filters.push_back(new ROS_EMAfilter(nh, inputTopics[i].c_str(), outputTopics[i].c_str(), smoothingFactors[i]));
    }

    ROS_INFO("node setup complete.");
    ros::spin();

    for(ROS_EMAfilter* ptr : filters)
    {
        delete ptr;
    }
    return 0;
}