#include "pure_pursuit/lookahead_point_generator.hpp"
#include "pure_pursuit/load_centerline_from_csv.hpp"

#define LOOKAHEAD 5

PoiPlanner::PoiPlanner(ros::NodeHandle& nh, const std::string& centerline_file) : nh_(nh) {
    // Advertise publisher
    pub_ = nh_.advertise<geometry_msgs::Point>("/lookahead_point", 10);

    // Create subscriber
    index_sub_ = nh_.subscribe("/projected_point_index", 10, &PoiPlanner::indexCallback, this);

    // Load centerline from CSV file
    centerline_ = loadCenterlineFromCSV(centerline_file);
}

void PoiPlanner::run() {
    ros::spin();
}

void PoiPlanner::indexCallback(const std_msgs::Int16::ConstPtr& msg) {

    int index = msg->data;
    
    // DEBUG
    std::cout << "this is current car position\n";
    std::cout << centerline_[index].x << ", " << centerline_[index].y << "\n\n";
    // DEBUG

    // Set and publish POI
    int poi_centerline_point_iterator = index + LOOKAHEAD;
    if (poi_centerline_point_iterator >= centerline_.size()) {
        poi_centerline_point_iterator -= centerline_.size();
    }
    geometry_msgs::Point poi_msg;
    poi_msg.x = centerline_[poi_centerline_point_iterator].x;
    poi_msg.y = centerline_[poi_centerline_point_iterator].y;

    pub_.publish(poi_msg);

    ROS_INFO("Lookahead Point: (%f, %f)", poi_msg.x, poi_msg.y);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "lookahead_point_generator");

    if (argc < 2) {
        ROS_ERROR("Usage: rosrun pure_pursuit lookahead_point_generator <centerline_file>");
        return 1;
    }

    ros::NodeHandle nh;
    std::string centerline_file = argv[1];
    PoiPlanner planner(nh, centerline_file);
    planner.run();

    return 0;
}

