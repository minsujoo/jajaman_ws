#include "ros/ros.h"
#include "geometry_msgs/Point.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "projected_point_publisher");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<geometry_msgs::Point>("/projected_point", 10);

    ros::Rate loop_rate(1); // 1 Hz로 메시지를 퍼블리시합니다.

    while (ros::ok()) {
        geometry_msgs::Point point;
        point.x = -10.147;
        point.y = 0.56409;
        point.z = 0.0;

        pub.publish(point);

        ROS_INFO("Published projected point: (%f, %f, %f)", point.x, point.y, point.z);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

