/*
1. Subscribe vehicle position by '/pf/pose/odom'
2. Find the closest point from the vehicle position among the points of 'centerline.csv'
3. Publish index of the closest point by '/projected_point_index"
*/

#include "centerline_projector/centerline_projector_node.hpp"

ReferencePointsNode::ReferencePointsNode(ros::NodeHandle& nh, const std::string& csv_file)
{
    loadReferencePoints(csv_file);
    kd_tree_ = new KDTree(reference_points_);
    closest_point_pub_ = nh.advertise<std_msgs::Int16>("/projected_point_index", 10);
    point_sub_ = nh.subscribe("/pf/pose/odom", 10, &ReferencePointsNode::pointCallback, this);
}

void ReferencePointsNode::pointCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    int closest_point_index = kd_tree_->findClosestPoint(msg->pose.pose.position.x, msg->pose.pose.position.y);

    std_msgs::Int16 closest_point_msg;
    closest_point_msg.data = closest_point_index;
    
    std::cout << "closest point index is " << closest_point_index << "\n";

    closest_point_pub_.publish(closest_point_msg);
}

KDTree::KDTree(const std::vector<std::pair<double, double>>& points)
    : reference_points_(points)
{
    std::vector<std::pair<std::pair<double, double>, int>> indexed_points;
    for (size_t i = 0; i < points.size(); ++i) {
        indexed_points.emplace_back(points[i], i);
    }
    root_ = buildTree(indexed_points, 0);
}

int KDTree::findClosestPoint(double x, double y)
{
    Node* best = nullptr;
    double best_dist = std::numeric_limits<double>::max();
    searchNearest(root_, x, y, 0, best, best_dist);
    return best->index;
}

KDTree::Node* KDTree::buildTree(std::vector<std::pair<std::pair<double, double>, int>>& points, int depth)
{
    return buildTreeHelper(points.begin(), points.end(), depth);
}

KDTree::Node* KDTree::buildTreeHelper(std::vector<std::pair<std::pair<double, double>, int>>::iterator start,
                                      std::vector<std::pair<std::pair<double, double>, int>>::iterator end, int depth)
{
    if (start == end)
        return nullptr;

    size_t len = std::distance(start, end);
    size_t mid = len / 2;
    auto nth = start + mid;
    auto cmp = [depth](const std::pair<std::pair<double, double>, int>& a, const std::pair<std::pair<double, double>, int>& b) {
        return depth % 2 == 0 ? a.first.first < b.first.first : a.first.second < b.first.second;
    };

    std::nth_element(start, nth, end, cmp);
    Node* node = new Node(nth->first, nth->second);
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
    ros::init(argc, argv, "centerline_projector_node");
    ros::NodeHandle nh;

    std::string csv_file;
    nh.param<std::string>("csv_file", csv_file, "~/jajaman_ws/centerline.csv");

    // ReferencePointsNode reference_points_node(nh, csv_file);
    ReferencePointsNode reference_points_node(nh, "centerline.csv");

    ros::spin();

    return 0;
}