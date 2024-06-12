#include <ros/ros.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <vector>
#include <iostream>

// 쿼터니언을 오일러 각도로 변환하는 함수
void quaternionToEuler(double x, double y, double z, double w, double &roll, double &pitch, double &yaw) {
    tf::Quaternion q(x, y, z, w);
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);
}

// 로컬 좌표계를 글로벌 좌표계로 변환하는 함수
std::vector<std::pair<double, double>> localToGlobal(const std::vector<std::pair<double, double>> &lidar_points, 
                                                     double x_v, double y_v, double yaw) {
    std::vector<std::pair<double, double>> global_points;
    for (const auto &point : lidar_points) {
        double x_l = point.first;
        double y_l = point.second;
        
        double x_g = x_v + (x_l * cos(yaw) - y_l * sin(yaw));
        double y_g = y_v + (x_l * sin(yaw) + y_l * cos(yaw));
        
        global_points.emplace_back(x_g, y_g);
    }
    return global_points;
}

// 글로벌 좌표계를 맵 좌표계로 변환하는 함수
std::vector<std::pair<double, double>> globalToMap(const std::vector<std::pair<double, double>> &global_points, 
                                                   const std::vector<double> &map_origin, double map_resolution) {
    std::vector<std::pair<double, double>> map_points;
    for (const auto &point : global_points) {
        double x_g = point.first;
        double y_g = point.second;
        
        double x_m = (x_g - map_origin[0]) / map_resolution;
        double y_m = (y_g - map_origin[1]) / map_resolution;
        
        map_points.emplace_back(x_m, y_m);
    }
    return map_points;
}

// 콜백 함수: Odometry 메시지를 처리
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    // 포즈 정보 추출
    double pose_position_x = msg->pose.pose.position.x;
    double pose_position_y = msg->pose.pose.position.y;
    double pose_orientation_x = msg->pose.pose.orientation.x;
    double pose_orientation_y = msg->pose.pose.orientation.y;
    double pose_orientation_z = msg->pose.pose.orientation.z;
    double pose_orientation_w = msg->pose.pose.orientation.w;
    
    // 쿼터니언을 오일러 각도로 변환
    double roll, pitch, yaw;
    quaternionToEuler(pose_orientation_x, pose_orientation_y, pose_orientation_z, pose_orientation_w, roll, pitch, yaw);
    
    // 라이다 포인트 예시 데이터 (로컬 좌표계에서)
    std::vector<std::pair<double, double>> lidar_points = { {1, 2}, {3, 4}, {-1, -1} };
    
    // 로컬 좌표계를 글로벌 좌표계로 변환
    std::vector<std::pair<double, double>> global_points = localToGlobal(lidar_points, pose_position_x, pose_position_y, yaw);
    
    // 맵 정보
    double map_resolution = 0.05;  // meters per pixel
    std::vector<double> map_origin = { -27.897049, -10.435905, 0.0 };  // origin of the map in global coordinates
    
    // 글로벌 좌표계를 맵 좌표계로 변환
    std::vector<std::pair<double, double>> map_points = globalToMap(global_points, map_origin, map_resolution);
    
    // 결과 출력
    for (const auto &point : map_points) {
        std::cout << "Map Point: (" << point.first << ", " << point.second << ")" << std::endl;
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "lidar_to_map_coordinates");
    ros::NodeHandle nh;
    
    // Odometry 메시지를 구독
    ros::Subscriber odom_sub = nh.subscribe("/pf/pose/odom", 1000, odomCallback);
    
    ros::spin();
    
    return 0;
}
