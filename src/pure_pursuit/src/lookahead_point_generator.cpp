/*
lookahead point generator
It gets POINT_GAP as constant, which is the gap distance between each centerline points
Lookahead distance is set by speed. (proportional)
*/

#include "pure_pursuit/lookahead_point_generator.hpp"
#include "pure_pursuit/load_centerline_from_csv.hpp"

#define POINT_GAP 0.323969627
// #define LOOKAHEAD_DIST_PER_SPEED 0.7
#define DEFAULT_LOOKAHEAD_DIST 0.5
#define LOOKAHEAD_DIST_PER_SPEED 0.5
// #define LOOKAHEAD 10
#define SPEED_TURNING_POINT 2
#define LOOKAHEAD_DIST_PER_SPEED_2 2.0



//! manual lookahead index set by index
#define IDX_FROM_1 137
#define IDX_TO_1 145
#define LOOKAHEAD_IDX_1 3

#define IDX_FROM_2 50
#define IDX_TO_2 60
#define LOOKAHEAD_IDX_2 12


// index 137 ~ 147 구간은 lookahead index = 3

PoiPlanner::PoiPlanner(ros::NodeHandle& nh, const std::string& centerline_file) : nh_(nh) {
    // Initial speed_
    speed_ = -1;

    // Advertise publisher
    pub_ = nh_.advertise<geometry_msgs::Point>("/lookahead_point", 10);

    // Create subscriber
    index_sub_ = nh_.subscribe("/projected_point_index", 10, &PoiPlanner::indexCallback, this);
    speed_sub_ = nh_.subscribe("/speed", 10, &PoiPlanner::speedCallback, this);

    // Load centerline from CSV file
    centerline_ = loadCenterlineFromCSV(centerline_file);
}

void PoiPlanner::run() {
    ros::spin();
}

void PoiPlanner::speedCallback(const std_msgs::Float64::ConstPtr& msg) {
    speed_ = msg->data;
}

void PoiPlanner::indexCallback(const std_msgs::Int16::ConstPtr& msg) {

    int index = msg->data;
    
    // DEBUG
    std::cout << "this is current car position\n";
    std::cout << centerline_[index].x << ", " << centerline_[index].y << "\n\n";
    // DEBUG

    // Set and publish POI
    //! here is important!
    // if speed is not subscribed yet. default lookahead dist is 1
    double lookahead_dist = 1;
    if (speed_ != -1) {
        if (speed_ > 3.5) speed_ = 3.5;  //! for saturate speed
        if (speed_ < SPEED_TURNING_POINT) {
            lookahead_dist = DEFAULT_LOOKAHEAD_DIST + speed_ * LOOKAHEAD_DIST_PER_SPEED;
        }
        else {
            lookahead_dist = DEFAULT_LOOKAHEAD_DIST + SPEED_TURNING_POINT * LOOKAHEAD_DIST_PER_SPEED + (speed_ - SPEED_TURNING_POINT) * LOOKAHEAD_DIST_PER_SPEED_2;
        }
    }
    int lookahead_index = lookahead_dist / POINT_GAP;
    //! manual setting
    if (index >= IDX_FROM_1 && index <= IDX_TO_1) {
        lookahead_dist = -1;
        lookahead_index = LOOKAHEAD_IDX_1;
    }
    else if (index >= IDX_FROM_2 && index <= IDX_TO_2) {
        lookahead_dist = -1;
        lookahead_index = LOOKAHEAD_IDX_2;
    }
    //!
    int poi_centerline_point_iterator = index + lookahead_index;
    std::cout << "speed is : " << speed_ << "\n";
    std::cout << "lookahead distance is : " << lookahead_dist << "\n";
    std::cout << "lookahead index is : " << lookahead_index << "\n";
    std::cout << "point gap btw indices is : " << POINT_GAP << "\n";
    // int poi_centerline_point_iterator = index + LOOKAHEAD;
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

