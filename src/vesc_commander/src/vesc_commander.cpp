#include <ros/ros.h>
#include <cmath>
#include <mutex>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

#define DEFAULT_STEER_OFFSET -2.5 // -2.5

#define IDX_FROM_1 80
#define IDX_TO_1 90
#define STEER_OFFSET_1 -5

#define IDX_FROM_2 0
#define IDX_TO_2 5
#define STEER_OFFSET_2 -10.0

#define IDX_FROM_3 98   //! 100
#define IDX_TO_3 105
#define STEER_OFFSET_3 7.0

#define IDX_FROM_4 30
#define IDX_TO_4 33
#define STEER_OFFSET_4 -4

#define IDX_FROM_5 34
#define IDX_TO_5 40
#define STEER_OFFSET_5 -12

#define IDX_FROM_6 41
#define IDX_TO_6 50
#define STEER_OFFSET_6 -15

#define IDX_FROM_7 62
#define IDX_TO_7 70
#define STEER_OFFSET_7 -15

#define IDX_FROM_8 72
#define IDX_TO_8 77
#define STEER_OFFSET_8 15

#define IDX_FROM_9 95
#define IDX_TO_9 97
#define STEER_OFFSET_9 15

#define HIGHIST_FREQUENCY 100
// #define CONSTANT_SPEED_MODE 0  // 0: various speed, 1: constant speed
// #define CONSTANT_SPEED 1.8     // valid only when it is constant speed mode

// mode, const speed
bool mode;
double const_speed;

// publisher, msg, mutex
ros::Publisher drive_pub;
std::mutex m_steer;
std::mutex m_speed;
std::mutex m_e_stop;
std::mutex m_index;

// steer, speed, steer_ok, speed_ok, e_stop, index, steer_offset
double steer;
double speed;
bool steer_ok;
bool speed_ok;
bool e_stop;
int idx;
double steer_offset;  //!

void indexCallback(const std_msgs::Int16::ConstPtr &msg)
{
    m_index.lock();
    idx = int(msg->data);
    // std::cout << "received index: " << index << "\n";
    m_index.unlock();
}

void eStopCallback(const std_msgs::Bool::ConstPtr &msg)
{
    m_e_stop.lock();
    e_stop = msg->data;
    std::cout << "received e_stop: " << e_stop << "\n";
    m_e_stop.unlock();
}

void steerCallback(const std_msgs::Float64::ConstPtr &msg)
{
    m_steer.lock();
    steer_ok = true;
    steer = msg->data;
    m_steer.unlock();
}

void speedCallback(const std_msgs::Float64::ConstPtr &msg)
{
    m_speed.lock();
    speed_ok = true;
    speed = msg->data;
    m_speed.unlock();
}

void schedulerCallback(const ros::TimerEvent& event) {
    if (idx >= IDX_FROM_1 && idx <= IDX_TO_1) steer_offset = STEER_OFFSET_1;
    else if (idx >= IDX_FROM_2 && idx <= IDX_TO_2) steer_offset = STEER_OFFSET_2;
    else if (idx >= IDX_FROM_3 && idx <= IDX_TO_3) steer_offset = STEER_OFFSET_3;
    else if (idx >= IDX_FROM_4 && idx <= IDX_TO_4) steer_offset = STEER_OFFSET_4;
    else if (idx >= IDX_FROM_5 && idx <= IDX_TO_5) steer_offset = STEER_OFFSET_5;
    else if (idx >= IDX_FROM_6 && idx <= IDX_TO_6) steer_offset = STEER_OFFSET_6;
    else if (idx >= IDX_FROM_7 && idx <= IDX_TO_7) steer_offset = STEER_OFFSET_7;
    else if (idx >= IDX_FROM_8 && idx <= IDX_TO_8) steer_offset = STEER_OFFSET_8;
    else if (idx >= IDX_FROM_9 && idx <= IDX_TO_9) steer_offset = STEER_OFFSET_9;
    else steer_offset = DEFAULT_STEER_OFFSET;

    if (mode) {
        if (steer_ok) {
            ackermann_msgs::AckermannDriveStamped drive_msg;
            
            m_steer.lock();
            // std::cout << "ADDED " << STEER_OFFSET / 180 * M_PI << "\n";
            std::cout << "steer was " << steer << "\n";
            double corrected_steer = double(steer) + (steer_offset / 180 * M_PI);   //! add steering offset
            // double steer2 = steer - 0.0262;
            std::cout << "steer is now " << steer << "\n";
            std::cout << "corrected steer is now " << corrected_steer << "\n";
            drive_msg.drive.steering_angle = corrected_steer * (-1);
            m_steer.unlock();

            m_e_stop.lock();
            if (e_stop) drive_msg.drive.speed = 0;
            else drive_msg.drive.speed = const_speed;
            m_e_stop.unlock();

            drive_msg.header.stamp = ros::Time::now();
            drive_pub.publish(drive_msg);

            std::cout << "e stop           : " << e_stop << "\n";
            std::cout << "published!\n";
            std::cout << "speed (constant) : " << drive_msg.drive.speed << "\n";
            std::cout << "steer angle      : " << drive_msg.drive.steering_angle * (-1) << "\n";
            std::cout << "steer angle(deg) : " << drive_msg.drive.steering_angle * (-1) / M_PI * 180.0 << "\n";
            std::cout << "\n\n";
        }
        else {
            std::cout << "steer is not received yet!\n";
            // std::cout << "speed is constant. (ok)\n";
        }
    }
    else {
        if (steer_ok && speed_ok) {
            ackermann_msgs::AckermannDriveStamped drive_msg;

            m_e_stop.lock();
            if (e_stop) drive_msg.drive.speed = 0;
            else {
                m_speed.lock();
                drive_msg.drive.speed = speed;
                m_speed.unlock();
            }
            m_e_stop.unlock();

            m_steer.lock();
            // std::cout << "ADDED " << STEER_OFFSET / 180 * M_PI << "\n";
            std::cout << "steer was " << steer << "\n";
            double corrected_steer = steer + (steer_offset / 180 * M_PI);   //! add steering offset
            // double steer2 = steer - 0.0262;
            std::cout << "steer is now " << steer << "\n";
            std::cout << "corrected steer is now " << corrected_steer << "\n";
            drive_msg.drive.steering_angle = corrected_steer * (-1);
            m_steer.unlock();

            drive_msg.header.stamp = ros::Time::now();
            drive_pub.publish(drive_msg);

            std::cout << "e stop           : " << e_stop << "\n";
            std::cout << "published!\n";
            std::cout << "speed            : " << drive_msg.drive.speed << "\n";
            std::cout << "steer angle      : " << drive_msg.drive.steering_angle * (-1) << "\n";
            std::cout << "steer angle(deg) : " << drive_msg.drive.steering_angle * (-1) / M_PI * 180.0 << "\n";
            std::cout << "\n\n";
        }
        else {
            if (steer_ok == false) std::cout << "steer is not received yet!\n"; 
            if (speed_ok == false) std::cout << "speed is not received yet!\n";
        }
    }
}

int main(int argc, char** argv)
{
    //! steer offset
    steer_offset = 0;

    // Initialize the ROS node
    ros::init(argc, argv, "vesc_commander");
    ros::NodeHandle nh;

    // Initialize triggers
    steer_ok = false;
    speed_ok = false;
    e_stop = false;

    // Set Mode
    int input;
    bool valid_input = false;
    while (!valid_input) {
        // std::cout << "Please enter 0(false) or 1(true) for mode\n";
        // std::cout << "0: subscribe speed from speed_decider\n";
        // std::cout << "1: constant speed\n";
        // std::cin >> input;
        input = 0; //* ONLY MODE 0
        if (input == 0 || input == 1) {
            valid_input = true;
        }
        else {
            std::cout << "Invalid input.\n\n";
        }
    }
    mode = static_cast<bool>(input);

    const_speed = 0;
    bool valid_input2 = false;
    if (mode == true) {
        std::cout << "Please enter speed.\n";
        std::cin >> const_speed;
    }

    // Initialize scheduler
    ros::Timer scheduler = nh.createTimer(ros::Duration(1/HIGHIST_FREQUENCY), &schedulerCallback);

    // Define(advertise) publishers
    drive_pub = nh.advertise<ackermann_msgs::AckermannDriveStamped>("/vesc/ackermann_cmd", 10);

    // Subscribe topic
    ros::Subscriber e_stop_sub = nh.subscribe("/e_stop", 10, eStopCallback);
    ros::Subscriber steer_sub = nh.subscribe("/steer", 10, steerCallback);
    ros::Subscriber speed_sub = nh.subscribe("/speed", 10, speedCallback);
    ros::Subscriber index_sub = nh.subscribe("/projected_point_index", 10, indexCallback);

    // Spin
    ros::spin();

    return 0;
}
