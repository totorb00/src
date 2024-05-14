#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <laser_geometry/laser_geometry.h>

class laser_filtered
{
    
public:
    laser_filtered(){
        laser_filter = nh.subscribe<sensor_msgs::LaserScan>("/scan", 10, &laser_filtered::scan_callback, this);
        filtered_scan_pub = nh.advertise<sensor_msgs::LaserScan>("/scan/filtered", 1);
        point = nh.advertise<sensor_msgs::PointCloud2>("/scan/points", 1);
    }

    void scan_callback(const sensor_msgs::LaserScan::ConstPtr& input){
        sensor_msgs::LaserScan filtered_scan = *input;
        
        // Define the angle ranges to be filtered
        std::vector<std::pair<double, double>> angle_ranges = {
            {0.0, 0.45},  // hoid dugui
            {0.9, 1.35},  //baruun hoid dugui
            {1.85, 2.1},   //baruun urd
            {3.0, 3.15},
            {-3.15, -3.0}//-utga avah bol - ih toonos -baga tooruu
        };

        // Filter points within the specified angle ranges
        for (auto& range : angle_ranges) {
            applyFilter(input, filtered_scan, range.first, range.second);
        }
        sensor_msgs::PointCloud2 cloud;
        projector.projectLaser(filtered_scan, cloud);//laser scan into point cloud section
        cloud.header.frame_id = "map";//Just change it whatever. Which frame do you use.
        point.publish(cloud);
        // Publish the filtered scan
        // filtered_scan_pub.header.frame_id = "map";
        filtered_scan_pub.publish(filtered_scan);
    }

    void applyFilter(const sensor_msgs::LaserScan::ConstPtr& input, sensor_msgs::LaserScan& filtered_scan, double lower_angle, double upper_angle) {
        for (size_t i = 0; i < input->ranges.size(); ++i) {
            double angle = input->angle_min + i * input->angle_increment;
            if (angle >= lower_angle && angle <= upper_angle) {
                filtered_scan.ranges[i] = std::numeric_limits<float>::infinity();
            }
        }
    }

protected:
    ros::NodeHandle nh;
    ros::Subscriber laser_filter;
    ros::Publisher filtered_scan_pub;
    ros::Publisher point;
    laser_geometry::LaserProjection projector; 
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "wheel_filtered_node");
    ROS_INFO("Started wheel_filter");
    laser_filtered node;
    ros::spin();
    return 0;
}
