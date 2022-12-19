#include "ros/ros.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "XmlRpcValue.h"

#include "../include/ema_filter.h"

class ROS_EMAtrajectoryFilter
{
    std::vector<EMAfilter*> m_filters;
    ros::Publisher m_publisher;
    ros::Subscriber m_subscriber;

    public:

    ROS_EMAtrajectoryFilter(ros::NodeHandle& nh, const char* inputTopic, const char* outputTopic, std::vector<float>& smoothingFactors){
        
        m_publisher = nh.advertise<trajectory_msgs::JointTrajectory>(outputTopic, 5);
        m_subscriber = nh.subscribe(inputTopic, 5, &ROS_EMAtrajectoryFilter::callback, this);

        std::stringstream infoMsg;
        infoMsg << "raw:/" << inputTopic << " | filtered:/" << outputTopic << " | smoothing Factors:";

        for(float smoothingFactor : smoothingFactors){
            m_filters.push_back(new EMAfilter(smoothingFactor));
            infoMsg << " " << smoothingFactor;
        }
        ROS_INFO("%s", infoMsg.str().c_str());
    }

    ~ROS_EMAtrajectoryFilter(){
        for(EMAfilter *x : m_filters){
            delete x;
        }
    }

    void callback(trajectory_msgs::JointTrajectory trajectoryMsg)
    {
        std::vector<double> jointValues = trajectoryMsg.points[0].positions;

        for(int i=0; i<jointValues.size(); i++)
        {
            jointValues[i] = m_filters[i]->filterValue(jointValues[i]);
        }
        trajectoryMsg.points[0].positions = jointValues;
        m_publisher.publish(trajectoryMsg);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ema_trajectory_filter_node");
    ros::NodeHandle nh;

    if(!ros::param::has("~smoothingFactors")){
        ROS_ERROR("private param <smoothingFactors> not found\nThis is a list of lists of floats\neach topic will have its own smoothing factors equal to number of joints.");
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
    std::vector<std::vector<float>> smoothingFactors;

    ros::param::get("~inputTopics", inputTopics);
    ros::param::get("~outputTopics", outputTopics);

    XmlRpc::XmlRpcValue list;
    ros::param::get("~smoothingFactors",list);
    ROS_ASSERT(list.getType() == XmlRpc::XmlRpcValue::TypeArray);

    for(int i = 0; i < list.size(); i++){
        ROS_ASSERT(list[i].getType() == XmlRpc::XmlRpcValue::TypeArray);

        std::vector<float> row;
        for(int j = 0; j < list[i].size(); j++){
            ROS_ASSERT(list[i][j].getType() == XmlRpc::XmlRpcValue::TypeDouble);

            row.push_back(static_cast<double>(list[i][j]));
        }
        smoothingFactors.push_back(row);
    }

    std::vector<ROS_EMAtrajectoryFilter*> filters;

    for(int i=0; i<inputTopics.size(); i++)
    {
        filters.push_back(new ROS_EMAtrajectoryFilter(nh, inputTopics[i].c_str(), outputTopics[i].c_str(), smoothingFactors[i]));
    }

    ROS_INFO("node setup complete.");
    ros::spin();

    for(ROS_EMAtrajectoryFilter* ptr : filters)
    {
        delete ptr;
    }
    return 0;
}