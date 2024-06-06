#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float64.h>
#include <vector>
#include <string>

/*
/scan
/pf/pose/odom (nav_msgs/Odometry)
/projected_position_index (std_msgs/Int16)
/lookahead_point (geometry_msgs/Pose)
/speed
centerline.csv
inner_wall.csv
outer_wall.csv
*/

#define FRAME_ID "map"

ros::Publisher p0_marker_pub;
ros::Publisher p1_marker_pub;
ros::Publisher p2_marker_pub;
ros::Publisher p3_marker_pub;
ros::Publisher p4_marker_pub;
ros::Publisher p5_marker_pub;

ros::Publisher path_pub;

std::vector<std::vector<double>> centerline;
std::vector<std::vector<double>> inner_wall;
std::vector<std::vector<double>> outer_wall;

void readCSV(const std::string& filename, std::vector<std::vector<double>>& points_storage) {
    std::ifstream file(filename);

    if (!file.is_open()) {
        std::cerr << "파일을 열 수 없습니다: " << filename << std::endl;
        return;
    }

    std::string line;
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string x_str, y_str;

        if (std::getline(ss, x_str, ',') && std::getline(ss, y_str, ',')) {
            double x = std::stod(x_str);
            double y = std::stod(y_str);
            points_storage.push_back({x, y});
        }
    }

    file.close();
}

/*
struct Quaternion {
    double x;
    double y;
    double z;
    double w;
};

Quaternion quaternion;

Quaternion headingToQuaternion(double heading) {
    Quaternion qt;
    
    // Convert heading to radians
    double yaw = heading * (-1) + M_PI/2;
    
    // Calculate half angles
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    
    // Set quaternion values
    qt.x = 0.0;
    qt.y = 0.0;
    qt.z = sy;
    qt.w = cy;
    
    return qt;
}

void headingCallback(const ac_car_states_msgs::Float64Stamped::ConstPtr& msg) {
    quaternion = headingToQuaternion(msg->data);
    // std::cout << "calculated quaternion!" << "\n";
    // std::cout << "quaternion.x: " << quaternion.x << "\n";
    // std::cout << "quaternion.y: " << quaternion.y << "\n";
    // std::cout << "quaternion.z: " << quaternion.z << "\n";
    // std::cout << "quaternion.w: " << quaternion.w << "\n";
}
*/

nav_msgs::Path path_msg;

//! /pf/pose/odom (nav_msgs/Odometry)
void p0Callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    // Make vehicle message
    visualization_msgs::Marker marker_msg;
    marker_msg.header.frame_id = FRAME_ID;
    marker_msg.ns = "pf_odom";
    marker_msg.id = 0;
    marker_msg.type = visualization_msgs::Marker::CUBE;
    marker_msg.action = visualization_msgs::Marker::ADD;
    marker_msg.pose = msg->pose.pose;
    // marker_msg.pose.position.x = msg->pose.pose.position.x;
    // marker_msg.pose.position.y = msg->pose.pose.position.y;
    // marker_msg.pose.position.z = 0;
    // marker_msg.pose.orientation.x = msg->pose.pose.orientation.x;
    // marker_msg.pose.orientation.y = msg->pose.pose.orientation.y;
    // marker_msg.pose.orientation.z = msg->pose.pose.orientation.z;
    // marker_msg.pose.orientation.w = msg->pose.pose.orientation.w;
    marker_msg.scale.x = 0.3;  //! 아마도 차량의 좌우 길이
    marker_msg.scale.y = 0.6;  //! 아마도 차량의 앞뒤 길이
    marker_msg.scale.z = 0.3;  //! 차량의 높이
    marker_msg.color.r = 0.0;
    marker_msg.color.g = 0.0;
    marker_msg.color.b = 1.0;
    marker_msg.color.a = 1.0;
    marker_msg.lifetime = ros::Duration(0.1);

    
    // Make path message
    path_msg.header.frame_id = FRAME_ID;
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = marker_msg.pose.position.x;
    pose.pose.position.y = marker_msg.pose.position.y;
    pose.pose.position.z = marker_msg.pose.position.z;
    path_msg.poses.push_back(pose);
    

    if(!ros::ok()) {
        return;
    }

    // Publish messages
    marker_msg.header.stamp = ros::Time::now();
    p0_marker_pub.publish(marker_msg);
    // std::cout << "published /vehicle" << "\n";
    
    path_msg.header.stamp = ros::Time::now();
    path_pub.publish(path_msg);
    // std::cout << "published /vehicle_path" << "\n\n";
}

//! /lookahead_point (geometry_msgs/Pose)
void p1Callback(const geometry_msgs::Pose::ConstPtr& msg)
{
    // Make vehicle message
    visualization_msgs::Marker marker_msg;
    marker_msg.header.frame_id = FRAME_ID;
    marker_msg.ns = "lookahead_point";
    marker_msg.id = 0;
    marker_msg.type = visualization_msgs::Marker::SPHERE;
    marker_msg.action = visualization_msgs::Marker::ADD;
    marker_msg.pose.position.x = msg->position.x;
    marker_msg.pose.position.y = msg->position.y;
    marker_msg.pose.position.z = 0;
    marker_msg.pose.orientation.x = 0.0;
    marker_msg.pose.orientation.y = 0.0;
    marker_msg.pose.orientation.z = 0.0;
    marker_msg.pose.orientation.w = 1.0;
    marker_msg.scale.x = 1.0;
    marker_msg.scale.y = 1.0;
    marker_msg.scale.z = 1.0;
    marker_msg.color.r = 0.0;
    marker_msg.color.g = 1.0;
    marker_msg.color.b = 0.0;
    marker_msg.color.a = 1.0;
    marker_msg.lifetime = ros::Duration(0.1);

    if(!ros::ok()) {
        return;
    }

    // Publish messages
    marker_msg.header.stamp = ros::Time::now();
    p1_marker_pub.publish(marker_msg);
}

void p2Callback(const ac_car_states_msgs::Point::ConstPtr& msg)
{
    // Make vehicle message
    visualization_msgs::Marker marker_msg;
    marker_msg.header.frame_id = FRAME_ID;
    marker_msg.ns = "lookahead";
    marker_msg.id = 0;
    marker_msg.type = visualization_msgs::Marker::SPHERE;
    marker_msg.action = visualization_msgs::Marker::ADD;
    marker_msg.pose.position.x = msg->x - 438000;
    marker_msg.pose.position.y = msg->y - 4206000;
    marker_msg.pose.position.z = 0;
    marker_msg.pose.orientation.x = 0.0;
    marker_msg.pose.orientation.y = 0.0;
    marker_msg.pose.orientation.z = 0.0;
    marker_msg.pose.orientation.w = 1.0;
    marker_msg.scale.x = 5.0;
    marker_msg.scale.y = 5.0;
    marker_msg.scale.z = 5.0;
    marker_msg.color.r = 0.0;
    marker_msg.color.g = 1.0;
    marker_msg.color.b = 0.0;
    marker_msg.color.a = 1.0;
    marker_msg.lifetime = ros::Duration(0.1);

    if(!ros::ok()) {
        return;
    }

    // Publish messages
    marker_msg.header.stamp = ros::Time::now();
    p2_marker_pub.publish(marker_msg);
}

void p3Callback(const ac_car_states_msgs::Point::ConstPtr& msg)
{
    // Make vehicle message
    visualization_msgs::Marker marker_msg;
    marker_msg.header.frame_id = FRAME_ID;
    marker_msg.ns = "p3";
    marker_msg.id = 0;
    marker_msg.type = visualization_msgs::Marker::SPHERE;
    marker_msg.action = visualization_msgs::Marker::ADD;
    marker_msg.pose.position.x = msg->x - 438000;
    marker_msg.pose.position.y = msg->y - 4206000;
    marker_msg.pose.position.z = 0;
    marker_msg.pose.orientation.x = 0.0;
    marker_msg.pose.orientation.y = 0.0;
    marker_msg.pose.orientation.z = 0.0;
    marker_msg.pose.orientation.w = 1.0;
    marker_msg.scale.x = 5.0;
    marker_msg.scale.y = 5.0;
    marker_msg.scale.z = 5.0;
    marker_msg.color.r = 0.0;
    marker_msg.color.g = 0.0;
    marker_msg.color.b = 1.0;
    marker_msg.color.a = 1.0;
    marker_msg.lifetime = ros::Duration(0.1);

    if(!ros::ok()) {
        return;
    }

    // Publish messages
    marker_msg.header.stamp = ros::Time::now();
    p3_marker_pub.publish(marker_msg);
}

void p4Callback(const ac_car_states_msgs::Point::ConstPtr& msg)
{
    // Make vehicle message
    visualization_msgs::Marker marker_msg;
    marker_msg.header.frame_id = FRAME_ID;
    marker_msg.ns = "p4";
    marker_msg.id = 0;
    marker_msg.type = visualization_msgs::Marker::SPHERE;
    marker_msg.action = visualization_msgs::Marker::ADD;
    marker_msg.pose.position.x = msg->x - 438000;
    marker_msg.pose.position.y = msg->y - 4206000;
    marker_msg.pose.position.z = 0;
    marker_msg.pose.orientation.x = 0.0;
    marker_msg.pose.orientation.y = 0.0;
    marker_msg.pose.orientation.z = 0.0;
    marker_msg.pose.orientation.w = 1.0;
    marker_msg.scale.x = 5.0;
    marker_msg.scale.y = 5.0;
    marker_msg.scale.z = 5.0;
    marker_msg.color.r = 0.0;
    marker_msg.color.g = 1.0;
    marker_msg.color.b = 1.0;
    marker_msg.color.a = 1.0;
    marker_msg.lifetime = ros::Duration(0.1);

    if(!ros::ok()) {
        return;
    }

    // Publish messages
    marker_msg.header.stamp = ros::Time::now();
    p4_marker_pub.publish(marker_msg);
}

void p5Callback(const ac_car_states_msgs::Point::ConstPtr& msg)
{
    // Make vehicle message
    visualization_msgs::Marker marker_msg;
    marker_msg.header.frame_id = FRAME_ID;
    marker_msg.ns = "p5";
    marker_msg.id = 0;
    marker_msg.type = visualization_msgs::Marker::SPHERE;
    marker_msg.action = visualization_msgs::Marker::ADD;
    marker_msg.pose.position.x = msg->x - 438000;
    marker_msg.pose.position.y = msg->y - 4206000;
    marker_msg.pose.position.z = 0;
    marker_msg.pose.orientation.x = 0.0;
    marker_msg.pose.orientation.y = 0.0;
    marker_msg.pose.orientation.z = 0.0;
    marker_msg.pose.orientation.w = 1.0;
    marker_msg.scale.x = 5.0;
    marker_msg.scale.y = 5.0;
    marker_msg.scale.z = 5.0;
    marker_msg.color.r = 0.0;
    marker_msg.color.g = 1.0;
    marker_msg.color.b = 0.0;
    marker_msg.color.a = 1.0;
    marker_msg.lifetime = ros::Duration(0.1);

    if(!ros::ok()) {
        return;
    }

    // Publish messages
    marker_msg.header.stamp = ros::Time::now();
    p5_marker_pub.publish(marker_msg);
}

int main(int argc, char** argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "any_point_visualizer");
    ros::NodeHandle nh;

    //! Read CSV
    readCSV("~/jajaman_ws/centerline.csv", centerline);
    readCSV("~/jajaman_ws/inner_wall.csv", inner_wall);
    readCSV("~/jajaman_ws/outer_wall.csv", outer_wall);

    // Create publisher
    p0_marker_pub = nh.advertise<visualization_msgs::Marker>("/visual_pf_odom", 10);
    p1_marker_pub = nh.advertise<visualization_msgs::Marker>("/visual_lookahead", 10);
    p2_marker_pub = nh.advertise<visualization_msgs::Marker>("/p2_marker", 10);
    p3_marker_pub = nh.advertise<visualization_msgs::Marker>("/p3_marker", 10);
    p4_marker_pub = nh.advertise<visualization_msgs::Marker>("/p4_marker", 10);
    p5_marker_pub = nh.advertise<visualization_msgs::Marker>("/p5_marker", 10);
    centerline_pub = nh.advertise<visualization_msgs::MarkerArray>("/visual_centerline", 10);
    inner_wall_pub = nh.advertise<visualization_msgs::MarkerArray>("/visual_inner_wall", 10);
    outer_wall_pub = nh.advertise<visualization_msgs::MarkerArray>("/visual_outer_wall", 10);
    path_pub = nh.advertise<nav_msgs::Path>("/visual_pf_path", 10);

    // Create subscriber
    // ros::Subscriber heading_sub = nh.subscribe("/heading", 10, headingCallback);
    ros::Subscriber safety_lookahead_sub = nh.subscribe("/safety_lookahead", 10, p0Callback);
    ros::Subscriber collision_point_sub = nh.subscribe("/collision_point", 10, p1Callback);
    ros::Subscriber lookahead_sub = nh.subscribe("/lookahead_point", 10, p2Callback);
    ros::Subscriber any_point_3_sub = nh.subscribe("/any_point_3", 10, p3Callback);
    ros::Subscriber any_point_4_sub = nh.subscribe("/any_point_4", 10, p4Callback);
    ros::Subscriber any_point_5_sub = nh.subscribe("/any_point_5", 10, p5Callback);

    ros::spin();

    return 0;
}