// #include <ros/ros.h>
// #include <iostream>
// #include <sensor_msgs/LaserScan.h>
// #include <std_msgs/Bool.h>
// #include <std_msgs/Int32.h>

// int min_range = 0;
// int max_range = 0;
// int back_side = 0;
// int front_side = 0;
// int left_side = 0;
// int right_side = 0;

// int distance_measure(float min, float max){
//     if(min>max){
//         max=min;
//         return max;
//     }else 
//         return min;

// }

// void laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg){
//     // int back_side = distance_measure(msg->ranges[510], msg->ranges[530]);
//     for(int i=510; i<530; i++){
//         back_side = distance_measure(msg->ranges[i], msg->ranges[i+1]);
//     }
//     std::cout<<"Ard zai: "<<back_side*100<<std::endl;
//     std_msgs::Bool ard_baih;
//     if(( back_side * 100) < 60){
//         ard_baih.data = true;
//         back.publish(ard_baih);
//     }else{
//         ard_baih.data = false;
//         back.publish(ard_baih);
//     }
//     for(int j=650; j<680; j++){
//         right_side = distance_measure(msg->ranges[j], msg->ranges[j+1]);
//     }
//     std_msgs::Int32 baruun_zai;
//     baruun_zai.data = right_side;
//     std::cout<<"baruun zai: "<<right_side*100<<std::endl;
//     right.publish(baruun_zai);
//     for(int k=780; k<790; k++){
//         front_side = distance_measure(msg->ranges[k], msg->ranges[k+1]);
//     }
//     std_msgs::Int32 urd_zai;
//     std::cout<<"Urd taliin zai: "<<front_side<<std::endl;
//     urd_zai.data = front_side;
//     front.publish(urd_zai);

// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "Back_side_checker");
//     ros::NodeHandle nh;
//     ros::Subscriber laser = nh.subscribe("/scan", 10, &laser_callback);
//     ros::Publisher back = nh.advertise<std_msgs::Bool>("/back_side", 10);
//     ros::Publisher front = nh.advertise<std_msgs::Int32>("/front_side", 10);
//     ros::Publisher right = nh.advertise<std_msgs::Int32>("/right_side", 10);
//     ros::spin();
//     return 0;
// }
#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>

ros::Publisher back_pub;
ros::Publisher front_pub;
ros::Publisher right_pub;

void laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    int back_side = 0;
    int front_side = 0;
    int right_side = 0;

    // Calculate distances for back, front, and right sides
    for(int i = 510; i < 530; i++) {
        back_side = std::min(back_side, static_cast<int>(msg->ranges[i] * 100)); // Convert to centimeters
    }

    for(int j = 650; j < 680; j++) {
        right_side = std::min(right_side, static_cast<int>(msg->ranges[j] * 100)); // Convert to centimeters
    }

    for(int k = 780; k < 790; k++) {
        front_side = std::min(front_side, static_cast<int>(msg->ranges[k] * 100)); // Convert to centimeters
    }

    // Publish back side status
    std_msgs::Bool back_msg;
    back_msg.data = (back_side < 60);
    back_pub.publish(back_msg);

    // Publish right side distance
    std_msgs::Int32 right_msg;
    right_msg.data = right_side;
    right_pub.publish(right_msg);

    // Publish front side distance
    std_msgs::Int32 front_msg;
    front_msg.data = front_side;
    front_pub.publish(front_msg);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "Back_side_checker");
    ros::NodeHandle nh;

    // Subscribe to laser scan topic
    ros::Subscriber laser_sub = nh.subscribe("/scan", 10, laser_callback);

    // Advertise publishers for back, front, and right sides
    back_pub = nh.advertise<std_msgs::Bool>("/back_side", 10);
    front_pub = nh.advertise<std_msgs::Int32>("/front_side", 10);
    right_pub = nh.advertise<std_msgs::Int32>("/right_side", 10);

    ros::spin();
    return 0;
}

