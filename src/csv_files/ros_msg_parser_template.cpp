#include <ros/ros.h>
#include <sub_msgs/SubMsgType.h>  // subscribing message type
#include <pub_msgs/PubMsgType.h>  // publishing message type

// Initialize publishers
ros::Publisher publisher1;
ros::Publisher publisher2;
ros::Publisher publisher3;

// Create msgs to publish
pub_msgs::PubMsgType pub_msg_1;
pub_msgs::PubMsgType pub_msg_2;
pub_msgs::PubMsgType pub_msg_3;

void subscribeCallback(const sub_msgs::SubMsgType::ConstPtr &msg)
{
    // Update msgs
    pub_msg1.data = (msg->vehicle_speed);
    pub_msg2.data = (msg->longitudinal_acceleration);
    pub_msg3.data = (msg->lateral_acceleration);

    // Input timestamp
    pub_msg1.header.stamp = ros::Time::now();
    pub_msg2.header.stamp = ros::Time::now();
    pub_msg3.header.stamp = ros::Time::now();

    // Publish
    publisher1.publish(pub_msg1);
    publisher2.publish(pub_msg2);
    publisher3.publish(pub_msg3);
}

int main(int argc, char** argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "parser_node");
    ros::NodeHandle nh;

    // Define(advertise) publishers
    publisher1 = nh.advertise<pub_msgs::PubMsgType>("/pub_topic1", 10);
    publisher2 = nh.advertise<pub_msgs::PubMsgType>("/pub_topic2", 10);
    publisher3 = nh.advertise<pub_msgs::PubMsgType>("/pub_topic3", 10);

    // Subscribe topic
    ros::Subscriber subscriber = nh.subscribe("/sub_topic", 10, subscribeCallback);

    // Spin
    ros::spin();

    return 0;
}