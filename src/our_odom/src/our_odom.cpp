/*
If '/projected_point_index' is changed abruptly, publish '/e_stop' for the vehicle emergency-stop.
*/

#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <cmath>
#include <nav_msgs/Odometry.h>  // subscribing message type
#include <std_msgs/Int16.h>  // subscribing message type
#include <std_msgs/Bool.h>  // publishing message type

#define THRE 10
#define CENTERLINE_PATH "centerline.csv"

// Initialize publishers
ros::Publisher publisher1;
// ros::Publisher publisher2;
// ros::Publisher publisher3;

// Create msgs to publish
// std_msgs::Bool e_stop_msg;
// pub_msgs::PubMsgType pub_msg_2;
// pub_msgs::PubMsgType pub_msg_3;

int index_num;

int prev_index;

void subscribeCallback(const std_msgs::Int16::ConstPtr &msg)
{
    int current_index = msg->data;
    
    if (prev_index == -1) {
        prev_index = current_index;
        return;
    }

    int index_gap = current_index - prev_index;
    int index_gap_2 = current_index + index_num - prev_index;
    if (std::abs(index_gap) > THRE && std::abs(index_gap_2) > THRE) {
        std_msgs::Bool e_stop_msg;
        e_stop_msg.data = true;
        publisher1.publish(e_stop_msg);
        std::cout << "published e_stop since index gap 1 is " << index_gap << "\n";
        std::cout << "published e_stop since index gap 2 is " << index_gap_2 << "\n";
    }

    // if (std::abs(index_gap_2) > THRE) {
    //     std_msgs::Bool e_stop_msg;
    //     e_stop_msg.data = true;
    //     publisher1.publish(e_stop_msg);
    //     std::cout << "published e_stop since index gap is " << index_gap_2 << "\n";
    // }

    prev_index = current_index;
    
    std::cout << "Driving Well!\n";
    std::cout << "index gap  : " << index_gap << "\n";
    std::cout << "index gap2 : " << index_gap_2 << "\n";
    std::cout << "(index gap 2 is used only when around starting line)\n\n";
}

bool isEmptyLine(const std::string& line) {
    std::istringstream iss(line);
    std::string word;
    while (iss >> word) {
        if (!word.empty()) {
            return false; // Line contains some data
        }
    }
    return true; // Line is empty or contains only whitespace
}

int countNonEmptyLines(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Unable to open file: " << filename << std::endl;
        return -1; // Return an error code
    }

    std::string line;
    int nonEmptyLineCount = 0;

    while (std::getline(file, line)) {
        if (!isEmptyLine(line)) {
            nonEmptyLineCount++;
        }
    }

    file.close();
    return nonEmptyLineCount;
}

int main(int argc, char** argv)
{
    prev_index = -1;

    // Read centerline index number
    std::string filename = CENTERLINE_PATH;
    std::cout << "Reading " << filename << " to see the number of index of centerline.\n";
    index_num = countNonEmptyLines(filename);
    if (index_num == -1) {
        std::cout << "Something error here maybe because of reading csv?" << std::endl;
    }
    else {
        std::cout << "Centerline index number is " << index_num << "\n";
    }


    // Initialize the ROS node
    ros::init(argc, argv, "our_odom");
    ros::NodeHandle nh;

    // Define(advertise) publishers
    publisher1 = nh.advertise<std_msgs::Bool>("/e_stop", 10);
    // publisher2 = nh.advertise<pub_msgs::PubMsgType>("/pub_topic2", 10);
    // publisher3 = nh.advertise<pub_msgs::PubMsgType>("/pub_topic3", 10);

    // Subscribe topic
    ros::Subscriber subscriber = nh.subscribe("/projected_point_index", 10, subscribeCallback);

    // Spin
    ros::spin();

    return 0;
}