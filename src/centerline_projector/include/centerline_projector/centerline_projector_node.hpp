#ifndef CENTERLINE_PROJECTOR_NODE_HPP
#define CENTERLINE_PROJECTOR_NODE_HPP

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Int16.h>
#include <vector>
#include <utility>
#include <limits>
#include <cmath>
#include <algorithm>
#include <nav_msgs/Odometry.h>

class KDTree
{
public:
    KDTree(const std::vector<std::pair<double, double>>& points);
    int findClosestPoint(double x, double y);

private:
    struct Node {
        std::pair<double, double> point;
        int index;
        Node* left;
        Node* right;
        Node(const std::pair<double, double>& pt, int idx) : point(pt), index(idx), left(nullptr), right(nullptr) {}
    };

    Node* root_;
    Node* buildTree(std::vector<std::pair<std::pair<double, double>, int>>& points, int depth);
    Node* buildTreeHelper(std::vector<std::pair<std::pair<double, double>, int>>::iterator start,
                          std::vector<std::pair<std::pair<double, double>, int>>::iterator end, int depth);
    void searchNearest(Node* root, double x, double y, int depth, Node*& best, double& best_dist);
    double distance(const std::pair<double, double>& p1, double x, double y);

    std::vector<std::pair<double, double>> reference_points_;
};

class ReferencePointsNode
{
public:
    ReferencePointsNode(ros::NodeHandle& nh, const std::string& csv_file);
    void pointCallback(const nav_msgs::Odometry::ConstPtr& msg);

private:
    ros::Publisher closest_point_pub_;
    ros::Subscriber point_sub_;
    std::vector<std::pair<double, double>> reference_points_;
    KDTree* kd_tree_;

    void loadReferencePoints(const std::string& csv_file);
};


#endif // CENTERLINE_PROJECTOR_NODE_HPP

/*
#ifndef CENTERLINE_PROJECTOR_NODE_HPP
#define CENTERLINE_PROJECTOR_NODE_HPP

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <vector>
#include <utility>
#include <nav_msgs/Odometry.h>

class KDTree
{
public:
    KDTree(const std::vector<std::pair<double, double>>& points);
    std::pair<double, double> findClosestPoint(double x, double y);

private:
    struct Node {
        std::pair<double, double> point;
        Node* left;
        Node* right;
        Node(const std::pair<double, double>& pt) : point(pt), left(nullptr), right(nullptr) {}
    };

    Node* root_;
    Node* buildTree(std::vector<std::pair<double, double>>& points, int depth);
    void searchNearest(Node* root, double x, double y, int depth, Node*& best, double& best_dist);
    double distance(const std::pair<double, double>& p1, double x, double y);

    Node* buildTreeHelper(std::vector<std::pair<double, double>>::iterator start, std::vector<std::pair<double, double>>::iterator end, int depth);
};

class ReferencePointsNode
{
public:
    ReferencePointsNode(ros::NodeHandle& nh, const std::string& csv_file);
    void pointCallback(const nav_msgs::Odometry::ConstPtr& msg);

private:
    ros::Publisher closest_point_pub_;
    ros::Subscriber point_sub_;
    std::vector<std::pair<double, double>> reference_points_;
    KDTree* kd_tree_;

    void loadReferencePoints(const std::string& csv_file);
};

#endif // CENTERLINE_PROJECTOR_NODE_HPP
*/
