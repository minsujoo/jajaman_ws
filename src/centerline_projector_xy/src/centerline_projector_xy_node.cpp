/*
1. Subscribe vehicle position by '/pf/pose/odom'
2. Find the closest point from the vehicle position among the points of 'centerline.csv'
3. Publish closest point by '/closest_point"
*/

#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "centerline_projector_xy/centerline_projector_xy_node.hpp"

ReferencePointsNode::ReferencePointsNode(ros::NodeHandle& nh, const std::string& csv_file)
{
    loadReferencePoints(csv_file);
    kd_tree_ = new KDTree(reference_points_);
    closest_point_pub_ = nh.advertise<geometry_msgs::Pose>("/closest_point", 10);
    point_sub_ = nh.subscribe("/pf/pose/odom", 10, &ReferencePointsNode::pointCallback, this);
}

void ReferencePointsNode::pointCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    auto closest_point = kd_tree_->findClosestPoint(msg->pose.pose.position.x, msg->pose.pose.position.y);

    geometry_msgs::Pose closest_point_msg;
    closest_point_msg.position.x = closest_point.first;
    closest_point_msg.position.y = closest_point.second;

    closest_point_pub_.publish(closest_point_msg);
    std::cout << "closest point is " << closest_point_msg.position.x << ", " << closest_point_msg.position.y << "\n";
}

KDTree::KDTree(const std::vector<std::pair<double, double>>& points)
{
    std::vector<std::pair<double, double>> pts = points;
    root_ = buildTree(pts, 0);
}

std::pair<double, double> KDTree::findClosestPoint(double x, double y)
{
    Node* best = nullptr;
    double best_dist = std::numeric_limits<double>::max();
    searchNearest(root_, x, y, 0, best, best_dist);
    return best->point;
}

KDTree::Node* KDTree::buildTree(std::vector<std::pair<double, double>>& points, int depth)
{
    return buildTreeHelper(points.begin(), points.end(), depth);
}

KDTree::Node* KDTree::buildTreeHelper(std::vector<std::pair<double, double>>::iterator start, std::vector<std::pair<double, double>>::iterator end, int depth)
{
    if (start == end)
        return nullptr;

    size_t len = std::distance(start, end);
    size_t mid = len / 2;
    auto nth = start + mid;
    auto cmp = [depth](const std::pair<double, double>& a, const std::pair<double, double>& b) {
        return depth % 2 == 0 ? a.first < b.first : a.second < b.second;
    };

    std::nth_element(start, nth, end, cmp);
    Node* node = new Node(*nth);
    node->left = buildTreeHelper(start, nth, depth + 1);
    node->right = buildTreeHelper(nth + 1, end, depth + 1);

    return node;
}

void KDTree::searchNearest(Node* root, double x, double y, int depth, Node*& best, double& best_dist)
{
    if (!root)
        return;

    double d = distance(root->point, x, y);
    if (d < best_dist) {
        best_dist = d;
        best = root;
    }

    double diff = (depth % 2 == 0 ? x - root->point.first : y - root->point.second);
    Node* near = diff < 0 ? root->left : root->right;
    Node* far = diff < 0 ? root->right : root->left;

    searchNearest(near, x, y, depth + 1, best, best_dist);
    if (diff * diff < best_dist)
        searchNearest(far, x, y, depth + 1, best, best_dist);
}

double KDTree::distance(const std::pair<double, double>& p1, double x, double y)
{
    return std::sqrt((p1.first - x) * (p1.first - x) + (p1.second - y) * (p1.second - y));
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "centerline_projector_xy_node");
    ros::NodeHandle nh;

    std::string csv_file;
    nh.param<std::string>("csv_file", csv_file, "~/jajaman_ws/centerline.csv");

    // ReferencePointsNode reference_points_node(nh, csv_file);
    ReferencePointsNode reference_points_node(nh, "centerline.csv");

    ros::spin();

    return 0;
}
