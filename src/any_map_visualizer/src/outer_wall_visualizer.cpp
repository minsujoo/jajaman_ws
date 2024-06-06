#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>

#define CENTERLINE_CSV_PATH "centerline.csv"
#define INNER_WALL_CSV_PATH "inner_wall.csv"
#define OUTER_WALL_CSV_PATH "outer_wall.csv"

#define FRAME_ID "map"

void readCSV(const std::string& filename, std::vector<std::vector<double>>& centerline) {
    std::ifstream file(filename);

    if (!file.is_open()) {
        ROS_ERROR("cannot open the file: %s", filename.c_str());
        return;
    }

    std::string line;
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string x_str, y_str;

        if (std::getline(ss, x_str, ',') && std::getline(ss, y_str, ',')) {
            double x = std::stod(x_str);
            double y = std::stod(y_str);
            centerline.push_back({x, y});
        }
    }

    file.close();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "outer_wall_visualizer");
    ros::NodeHandle nh;

    // CSV 파일 경로 설정
    std::string csv_file;
    nh.param<std::string>("csv_file", csv_file, OUTER_WALL_CSV_PATH);

    std::vector<std::vector<double>> centerline;
    readCSV(csv_file, centerline);

    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("/visual_outer_wall", 1);
    ros::Rate r(1);

    while (ros::ok()) {
        visualization_msgs::Marker points;
        points.header.frame_id = FRAME_ID;
        points.header.stamp = ros::Time::now();
        points.ns = "outer_wall";
        points.action = visualization_msgs::Marker::ADD;
        points.pose.orientation.w = 1.0;

        points.id = 0;
        points.type = visualization_msgs::Marker::POINTS;

        points.scale.x = 0.1;
        points.scale.y = 0.1;

        points.color.r = 1.0f;
        points.color.a = 1.0;

        for (const auto& point : centerline) {
            geometry_msgs::Point p;
            p.x = point[0];
            p.y = point[1];
            p.z = 0;

            points.points.push_back(p);
        }

        marker_pub.publish(points);
        ROS_INFO("published centerline");

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
