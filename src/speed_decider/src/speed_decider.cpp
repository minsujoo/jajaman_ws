/*
Decide speeds along the projected point indeces.
Subscribe '/projected_point_index' and publish '/speed'
*/

#include <ros/ros.h>
#include <std_msgs/Int16.h>  // /projected_point_index (sub)
#include <std_msgs/Float64.h>  // /speed (pub)

#define IDX_1 0
#define SPEED_1 2.3
#define IDX_2 10
#define SPEED_2 4.5
#define IDX_3 15
#define SPEED_3 7.5
#define IDX_4 30
#define SPEED_4 3
#define IDX_5 40
#define SPEED_5 2.5
#define IDX_6 44      //! 50
#define SPEED_6 4     //! 2.5
#define IDX_7 60        // 60
#define SPEED_7 2     // 2
#define IDX_8 67      // 70
#define SPEED_8 1.5
#define IDX_9 80
#define SPEED_9 1.5
#define IDX_10 83
#define SPEED_10 5
#define IDX_11 96
#define SPEED_11 2
#define IDX_12 100  
#define SPEED_12 1.8
#define IDX_13 110
#define SPEED_13 2.5
#define IDX_14 120
#define SPEED_14 1.5
#define IDX_15 125
#define SPEED_15 4
#define IDX_16 136
#define SPEED_16 1.8
#define IDX_17 146
#define SPEED_17 1.8
#define IDX_18 149
#define SPEED_18 2.3
// #define IDX_16 480
// #define SPEED_16 1
// #define IDX_17 490
// #define SPEED_17 7

// Initialize publishers
ros::Publisher pub;

// Create msgs to publish
std_msgs::Float64 pub_msg;

double interpolate(double x1, double y1, double x2, double y2, double x_new) {
    return (y2 - y1) / (x2 - x1) * (x_new - x1) + y1;
}

void subscribeCallback(const std_msgs::Int16::ConstPtr &msg)
{
    int idx = msg->data;
    double speed;
    if (idx < IDX_2) {
        speed = interpolate(IDX_1, SPEED_1, IDX_2, SPEED_2, idx);
    }
    else if (idx < IDX_3) {
        speed = interpolate(IDX_2, SPEED_2, IDX_3, SPEED_3, idx);
    }
    else if (idx < IDX_4) {
        speed = interpolate(IDX_3, SPEED_3, IDX_4, SPEED_4, idx);
    }
    else if (idx < IDX_5) {
        speed = interpolate(IDX_4, SPEED_4, IDX_5, SPEED_5, idx);
    }
    else if (idx < IDX_6) {
        speed = interpolate(IDX_5, SPEED_5, IDX_6, SPEED_6, idx);
    }
    else if (idx < IDX_7) {
        speed = interpolate(IDX_6, SPEED_6, IDX_7, SPEED_7, idx);
    }
    else if (idx < IDX_8) {
        speed = interpolate(IDX_7, SPEED_7, IDX_8, SPEED_8, idx);
    }
    else if (idx < IDX_9) {
        speed = interpolate(IDX_8, SPEED_8, IDX_9, SPEED_9, idx);
    }
    else if (idx < IDX_10) {
        speed = interpolate(IDX_9, SPEED_9, IDX_10, SPEED_10, idx);
    }
    else if (idx < IDX_11) {
        speed = interpolate(IDX_10, SPEED_10, IDX_11, SPEED_11, idx);
    }
    else if (idx < IDX_12) {
        speed = interpolate(IDX_11, SPEED_11, IDX_12, SPEED_12, idx);
    }
    else if (idx < IDX_13) {
        speed = interpolate(IDX_12, SPEED_12, IDX_13, SPEED_13, idx);
    }
    else if (idx < IDX_14) {
        speed = interpolate(IDX_13, SPEED_13, IDX_14, SPEED_14, idx);
    }
    else if (idx < IDX_15) {
        speed = interpolate(IDX_14, SPEED_14, IDX_15, SPEED_15, idx);
    }
    else if (idx < IDX_16) {
        speed = interpolate(IDX_15, SPEED_15, IDX_16, SPEED_16, idx);
    }
    else if (idx < IDX_17) {
        speed = interpolate(IDX_16, SPEED_16, IDX_17, SPEED_17, idx);
    }
    else if (idx <= IDX_18) speed = interpolate(IDX_17, SPEED_17, IDX_18, SPEED_18, idx);
    // else if (idx < IDX_16) {
    //     speed = interpolate(IDX_15, SPEED_15, IDX_16, SPEED_16, idx);
    // }
    // else if (idx < IDX_17) {
    //     speed = interpolate(IDX_16, SPEED_16, IDX_17, SPEED_17, idx);
    // }
    else {
        speed = 0;
    }

    // Update msgs
    pub_msg.data = speed;

    // Input timestamp
    // pub_msg.header.stamp = ros::Time::now();

    // Publish
    pub.publish(pub_msg);

    std::cout << "car point index is : " << idx << "\n";
    std::cout << "decided speed is   : " << speed << "\n\n";
}

int main(int argc, char** argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "speed_decider");
    ros::NodeHandle nh;

    // Define(advertise) publishers
    pub = nh.advertise<std_msgs::Float64>("/speed", 10);

    // Subscribe topic
    ros::Subscriber subscriber = nh.subscribe("/projected_point_index", 10, subscribeCallback);

    // Spin
    ros::spin();

    return 0;
}