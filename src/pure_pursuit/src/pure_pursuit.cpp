/*
Calculate steering angle to reach the lookahead point subscribed.
Wheelbase and weight to the steer is defined as constant.
*/

#include <cmath>
#include <mutex>
#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "std_msgs/Float64.h"
#include <nav_msgs/Odometry.h>
#include <iostream>

#define WHEELBASE 0.3
#define HIGHIST_FREQUENCY 100
#define STEER_WEIGHT 1.2

struct Point {
    double x;
    double y;
};

class Car {
public:
    Car() {
        wheelbase = WHEELBASE;
        heading = 0;
        position.x = 0;
        position.y = 0;
        heading_received_state = false;
        utm_received_state = false;
    }

    Point position;

    double getWheelbase() { return wheelbase; }
    double getHeading() { return heading; }
    double getEasting() { return position.x; }
    double getNorthing() { return position.y; }
    bool isHeadingReceived() { return heading_received_state; }
    bool isUtmReceived() { return utm_received_state; }
    
    void setHeading(double angle) { heading = angle; }
    void setPosition(double x, double y) {
        position.x = x;
        position.y = y;
    }
    void setHeadingState(bool state) { heading_received_state = state; }
    void setUtmState(bool state) { utm_received_state = state; }

private:
    double wheelbase;
    double heading;
    bool heading_received_state;
    bool utm_received_state;
};

Car car;
Point poi;
std::mutex mutex_car;

ros::Publisher pub;

void poiCallback(const geometry_msgs::Point::ConstPtr& msg) {
    poi.x = msg->x;
    poi.y = msg->y;
}

// void headingCallback(const geometry_msgs::Pose::ConstPtr& msg) {
//     mutex_car.lock();
//     car.setHeading(msg->data);
//     car.setHeadingState(true);
//     mutex_car.unlock();
// }

void poseCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    mutex_car.lock();
    car.setPosition(msg->pose.pose.position.x, msg->pose.pose.position.y);
    car.setUtmState(true);
    car.setHeadingState(true);

    // quat -> yaw
    double yaw = std::atan2(
        2 * ((msg->pose.pose.orientation.x * msg->pose.pose.orientation.y) + (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z)),
        msg->pose.pose.orientation.w*msg->pose.pose.orientation.w + msg->pose.pose.orientation.x*msg->pose.pose.orientation.x - msg->pose.pose.orientation.y*msg->pose.pose.orientation.y - msg->pose.pose.orientation.z*msg->pose.pose.orientation.z
    );
    yaw -= M_PI / 2.0;
    
    if (yaw > M_PI ) {
        yaw -= 2.0 * M_PI;
        } else if  (yaw < -M_PI ) {
           yaw += 2.0 * M_PI;
        }
    yaw = -1*yaw;
    car.setHeading(yaw);
    mutex_car.unlock();
}

void schedulerCallback(const ros::TimerEvent& event) {
    if (!(car.isHeadingReceived() && car.isUtmReceived())) {
        return;
    }

    mutex_car.lock();
    double car_poi_angle = std::atan2((poi.y-car.position.y), (poi.x-car.position.x));
    double car_poi_dist = std::sqrt( std::pow(poi.x - car.position.x, 2) + std::pow(poi.y - car.position.y, 2) );
    double target_steering_angle = std::atan2(2 * car.getWheelbase() * std::cos(car.getHeading() + car_poi_angle), car_poi_dist);
    // target_steering_angle = target_steering_angle / M_PI * 180;
    target_steering_angle *= STEER_WEIGHT;

    std_msgs::Float64 msg;
    msg.data = target_steering_angle;
    pub.publish(msg);

    std::cout << "\n\n\n";
    ROS_INFO("target steering angle : %f", target_steering_angle);
    ROS_INFO("Position        : (%f, %f)", car.position.x, car.position.y);
    ROS_INFO("Heading         : %f", car.getHeading());
    ROS_INFO("Lookahead Point : (%f, %f)", poi.x, poi.y);
    ROS_INFO("We should go %f more to x axis", poi.x - car.position.x);
    ROS_INFO("We should go %f more to y axis", poi.y - car.position.y);
    mutex_car.unlock();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "pure_pursuit_node");
    ros::NodeHandle nh;

    ros::Subscriber poi_sub = nh.subscribe<geometry_msgs::Point>("/lookahead_point", 10, poiCallback);
    ros::Subscriber pose_sub = nh.subscribe<nav_msgs::Odometry>("/pf/pose/odom", 10, poseCallback);
    // ros::Subscriber heading_sub = nh.subscribe<geometry_msgs::Pose>("/pose", 10, headingCallback);

    ros::Timer scheduler = nh.createTimer(ros::Duration(1.0 / HIGHIST_FREQUENCY), schedulerCallback);

    pub = nh.advertise<std_msgs::Float64>("/steer", 10);

    ros::spin();

    return 0;
}

