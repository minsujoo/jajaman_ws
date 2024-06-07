#include <ros/ros.h>
#include <mutex>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

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

// steer, speed, steer_ok, speed_ok, e_stop
double steer;
double speed;
bool steer_ok;
bool speed_ok;
bool e_stop;

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
    if (mode) {
        if (steer_ok) {
            ackermann_msgs::AckermannDriveStamped drive_msg;
            
            m_steer.lock();
            drive_msg.drive.steering_angle = steer * (-1);
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
            std::cout << "steer angle      : " << drive_msg.drive.steering_angle << "\n";
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
            drive_msg.drive.steering_angle = steer * (-1);
            m_steer.unlock();

            drive_msg.header.stamp = ros::Time::now();
            drive_pub.publish(drive_msg);

            std::cout << "e stop           : " << e_stop << "\n";
            std::cout << "published!\n";
            std::cout << "speed       : " << drive_msg.drive.speed << "\n";
            std::cout << "steer angle : " << drive_msg.drive.steering_angle << "\n";
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
        std::cout << "Please enter 0(false) or 1(true) for mode\n";
        std::cout << "0: subscribe speed from speed_decider\n";
        std::cout << "1: constant speed\n";
        std::cin >> input;
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

    // Spin
    ros::spin();

    return 0;
}
