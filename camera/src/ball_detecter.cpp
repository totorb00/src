#include <ros/ros.h>
#include "std_msgs/String.h"
#include <iostream>
#include <vector>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <cctype>

int percentage ;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "ball_detecter_node");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::String>("/camera_data", 1024);
    std_msgs::String string_buf;
    const char *fifoPath = "/tmp/myfifo"; // Replace this with your FIFO path
    int fd = open(fifoPath, O_RDONLY);
    if (fd == -1) {
        perror("Failed to open FIFO for reading");
        return 1;
    }
    
    ros::Rate r(100);
    while (ros::ok()) {

        char buffer[1024]; // Adjust the buffer size as per your requirement
        ssize_t bytesRead = read(fd, buffer, sizeof(buffer));
        if (bytesRead == -1) {
            perror("read");
            close(fd);
            return 1;
        }else{
            buffer[bytesRead] = '\0';
            string_buf.data = buffer;
            pub.publish(string_buf);
        }
        
        r.sleep();
    }
    close(fd);
    return 0;
}