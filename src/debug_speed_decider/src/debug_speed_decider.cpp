/*
Publish '/speed' with user input for 'vesc_commander' subscribe it and control the vehicle with user decided speed.
*/

#include <ros/ros.h>
// #include <std_msgs/Int16.h>  // /projected_point_index (sub)
#include <std_msgs/Float64.h>  // /speed (pub)

int main(int argc, char** argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "debug_speed_decider");
    ros::NodeHandle nh;

    // Initialize publishers
    ros::Publisher pub;

    // Define(advertise) publishers
    pub = nh.advertise<std_msgs::Float64>("/speed", 10);

    while(ros::ok()) {
        std::cout << "speed?\n";
        double speed = 0;
        std::cin >> speed;
        if (speed >= 10) {
            speed = 0;
            std::cout << "\nspeed is too high!!!  --->  speed is ZERO.\n";
        }
        std_msgs::Float64 speed_msg;
        speed_msg.data = speed;
        pub.publish(speed_msg);
        std::cout << "Published /speed as " << speed << " !!\n\n";
    }

    // Spin
    // ros::spin();

    return 0;
}