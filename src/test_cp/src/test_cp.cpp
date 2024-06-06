#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <vector>
#include <fstream>
#include <sstream>
#include <iostream>
#include <cmath>

// Initialize publishers
ros::Publisher pub;

// Create msgs to publish
geometry_msgs::Pose pub_msg;


// Function to read a CSV file into a vector of vector of doubles
std::vector<std::vector<double>> readCSV(const std::string& filename) {
    std::vector<std::vector<double>> data;
    std::ifstream file(filename);
    std::string line;

    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return data; // Return empty vector if file can't be opened
    }

    while (getline(file, line)) {
        std::istringstream s(line);
        std::string field;
        std::vector<double> row;

        while (getline(s, field, ',')) {
            row.push_back(stod(field)); // Convert string to double and add to row
        }

        data.push_back(row); // Add the row to the main vector
    }

    file.close();
    return data;
}


void subscribeCallback(const geometry_msgs::Pose::ConstPtr &msg)
{
    double car_x = msg->position.x;
    double car_y = msg->position.y;

    // Calculate projected position
    double projected_x = car_x;
    double projected_y = car_y;

    // Update msgs
    pub_msg.position.x = projected_x;
    pub_msg.position.y = projected_y;

    // Input timestamp
    // pub_msg.header.stamp = ros::Time::now();

    // Publish
    pub.publish(pub_msg);
}

// Quaterniond toQuaternion(double yaw, double pitch, double roll) // yaw (Z), pitch (Y), roll (X)
// {
//     //Degree to radius:
//     yaw = yaw * M_PI / 180;
//     pitch = pitch * M_PI / 180;
//     roll = roll * M_PI / 180;


//     // Abbreviations for the various angular functions
//     double cy = cos(yaw * 0.5);
//     double sy = sin(yaw * 0.5);
//     double cp = cos(pitch * 0.5);
//     double sp = sin(pitch * 0.5);
//     double cr = cos(roll * 0.5);
//     double sr = sin(roll * 0.5);

//     Quaterniond q;
//     q.w = cy * cp * cr + sy * sp * sr;
//     q.x = cy * cp * sr - sy * sp * cr;
//     q.y = sy * cp * sr + cy * sp * cr;
//     q.z = sy * cp * cr - cy * sp * sr;
//     return q;
// }

int main(int argc, char** argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "test_cp");
    ros::NodeHandle nh;

    std::cout << "This node is to test if the centerline_projector works well.\n";
    std::cout << "When you type the point(x, y), then I will publish it as /point and let you know the closest point of the waypoints searching with BruteForce\n";

    pub = nh.advertise<geometry_msgs::Pose>("/point", 10);

    for (int i = 0; i < 100; i++) {
        geometry_msgs::Pose msg;
        std::cout << "\nx?";
        std::cin >> msg.position.x;
        std::cout << "y?";
        std::cin >> msg.position.y;
        std::cout << "heading?";
        // double tmp_heading_deg;
        // std::cin >> tmp_heading_deg;
        // msg.orientation = toQuaternion(tmp_heading_deg, 0, 0);

        pub.publish(msg);
        std::cout << "pubed!!\n";
        double min_dist = 100000;
        double min_dist_x = 0;
        double min_dist_y = 0;
        auto data = readCSV("general_centerline.csv");
        for (int i = 0; i < data.size(); i++) {
            double xi = data[i][0];
            double yi = data[i][1];
            double tmp_dist = std::sqrt((std::pow((msg.position.x - xi), 2)) + std::pow((msg.position.y - yi), 2));
            if (tmp_dist < min_dist) {
                min_dist = tmp_dist;
                min_dist_x = xi;
                min_dist_y = yi;
            }
        }
        std::cout << "Answer is " << min_dist_x << ", " << min_dist_y << " \n";
    }

    auto data = readCSV("general_centerline.csv");
    for (int i = 0; i < data.size(); i++) {
        for (int j = 0; j < data[i].size(); j++) {
            std::cout << data[i][j] << " ";
        }
        std::cout << std::endl;
    }

    // Define(advertise) publishers
    pub = nh.advertise<geometry_msgs::Pose>("/projected_position", 10);

    // Subscribe topic
    ros::Subscriber subscriber = nh.subscribe("/position", 10, subscribeCallback);

    // Spin
    ros::spin();

    return 0;
}