// #include <main.h>
#include <iostream>
#include <cmath>
#include <string>
#include <queue>
#include <stdlib.h>
#include <stdio.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int8MultiArray.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Int8.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Int16.h>


int game_status = 5;//game_status = [1]:Robot is starting zone;
                    //              [2]:Robot is retry zone;    
                    //              [0]:Robot is stopped;
bool game_side  = 0;
int status = 0;
int prev_status = 0;
std::string ongo="blue";
int ehlel = 0 ;
float theta = 0;
int left_distance = 0;
int front_distance=0;
int right_distance=0;
int back_distance=0;
int distance=0;
bool start_cmd=false;
bool imu_moving = false;
std::string promt;
bool imu_status = false;


int nearest_ball=0;
int camera_point_x=300;
int camera_point_y=-415;
int shortest_distance_x=0;
int shortest_distance_y=0;
bool take_ball=false;
float shortest_theta=0;
int u,w;
int min_val=1000;

bool recieved_color=false;
bool robot_ball = false;

int16_t robot_x=0;
int16_t robot_y=0;
int16_t robot_theta=0;

int last_x = 0;
int last_y = 0;
int last_theta = 0;
int true_ball = 0;
int prev_theta = 0;

std::vector<int> silo{2, 2, 2, 4, 4, 4, 3, 3, 3};
int silo_dist = 0;

geometry_msgs::Twist vel_pub;
std_msgs::Int8 BLDC;
std_msgs::UInt16 dcPos;
std_msgs::Int16 reset_cordinate;

void velocity_publishing(int x, int y, int z){
    vel_pub.linear.x = x;
    vel_pub.linear.y = y;
    vel_pub.angular.z = z;
}
void printing_vel(int stat, int x, int y, int z){
    ROS_INFO("Game status is %d", stat);
    ROS_INFO("ROBOT hashlaganas[%d] zaita bna", x);
    ROS_INFO("ROBOT uragshaa [%d] zaitai bna.", y);
    ROS_INFO("ROBOT [%d] gradus hazaij bna.", z);
}
int PID(double kp, double ki, double kd, float setpoint, float processVariable, int max_speed, int min_speed)
{
    double error;
    double out;
    error=setpoint-processVariable;
    out=kp*error;
        if(out > max_speed)
        out = max_speed;
        if(out < -max_speed)
        out = -max_speed;
        if(out < min_speed && out>0){
            out = min_speed;
        }
        if(out>-min_speed && out<0){
            out = -min_speed;
        }
        return out;
}


void camera_detect(const std_msgs::String::ConstPtr& ball_detect)
{
    // // Split the read data by space
    std::istringstream iss(ball_detect->data);
    std::vector<std::string> tokens;
    std::string token;
    std::vector<int16_t> area;
    std::string color;
    std::vector<int> percentage;
    // std::string ongo = "blue";//ali tald togloh ve gedeg ongo
    int counter = 0;
    int min_val = 1000;
    // float theta_ball=0;
    
    while (std::getline(iss, token, ' ')) {
        tokens.push_back(token);
    }
    // ROS_WARN("Begin");
    // Process the tokens
    for (const auto& token : tokens) {
        try {    
            int f = std::stoi(token); // Convert token to int
            if(counter==0){              
                shortest_distance_x = f;
                counter = 1;    
            }else{
                shortest_distance_y = -f;
            }
            
        } catch (const std::invalid_argument& e) {
            counter = 0;
            color = token;
           // shortest_theta = atan2(shortest_distance_x - camera_point_x, shortest_distance_y - camera_point_y);
            // nearest_ball = shortest_distance_x - camera_point_x;
            // std::cout<< ongo << std::endl;
            nearest_ball = (shortest_distance_y-camera_point_y)/2 +  sqrt(pow(shortest_distance_x - camera_point_x, 2) + pow(shortest_distance_y - camera_point_y, 2));
            // std::cout<< nearest_ball << std::endl;
            if((ongo.compare(color)) == 0){
                // std::cout<< "ssssd-----------------"<<std::endl;
                take_ball = true;
                if(min_val >= nearest_ball){
                    u = shortest_distance_x;
                    w = shortest_distance_y;
                    min_val = nearest_ball;//herev manai togloh taliin bombog haragdsan uyed hamgin oirhon zaitai bombogruu yvna
                    std::cout<<u<<","<<w<<","<<min_val<<"-----"<<nearest_ball<<std::endl;   
                    // ROS_INFO("U = %d", u);
                    // ROS_INFO("W = %d", w);
                    // ROS_INFO("min_val = %d", min_val);
                    // ROS_WARN("Calculate min val");
                }
            }else{
                // take_ball = false;
                // u = camera_point_x;
                // w = camera_point_y;
            }
        }
    }
    // ROS_WARN("End");
    
}

void imu_callback(const std_msgs::Bool::ConstPtr& imu_data)
{
    // imu_status = imu_data->data; 
}

// LIDARIIN DATA IREH URD HOID BARUN ZUUN TALIN ZAIG HEMJIH
void wall_clk(const std_msgs::Int16MultiArray::ConstPtr& wall_data)
{
   left_distance = wall_data->data[0];
   right_distance =wall_data->data[1];
   theta=wall_data->data[2];
   front_distance =wall_data->data[3];
   back_distance = wall_data->data[4];

//    std::cout<<wall_data->data[5]<<std::endl;
   if(ongo=="red"){
        if(wall_data->data[4] < 60){
            ehlel = 1;
        }else{
            ehlel = 2;
        }
   }
   else{
        if(wall_data->data[3] < 60){
            ehlel = 1;
        }
        else{
            ehlel = 2;
        }
   }
}

// ROBOTIN BAIRLAL OLOH HAVTANGAAS IREH /encoder TOPIC
void encoder_call(const std_msgs::Int16MultiArray::ConstPtr& encoder_data){
    robot_theta = encoder_data->data[2];
    robot_x = encoder_data->data[3];
    robot_y = encoder_data->data[4];
}

void sensor_clk(const std_msgs::Int8MultiArray::ConstPtr& sensor_data)
{
    ROS_INFO("%d", sensor_data->data[0]);
    if(((sensor_data->data[0] & 0b00010000 == 0b00010000) || (sensor_data->data[0] & 0b00100000==0b00100000) )&& sensor_data->data[0] & 0b01000000 == 0b01000000    ){
        robot_ball = true;
    // if(sensor_data->data[0]==48){
    //     robot_ball = true;
    }else{
        robot_ball = false;
    }
    true_ball = sensor_data->data[1];// 0 = purple, 1 = blue, 2 = red.
                                    
}


// TALBAIN ALI TALD TOGLOJ BAIGAAGAA MEDEH HESEG STM32 OOS COLOR SENSORING UTGA IRNE
void color_ckeck(const std_msgs::String::ConstPtr& color_ck)
{
    if(!recieved_color){
        recieved_color = true;
        if(color_ck->data == "red"){
            ongo = color_ck->data.c_str();
            ROS_WARN("My team is starting from [%s]!!!", color_ck->data.c_str());
            // game_side  = false;
        }
        if(color_ck->data == "blue"){
            ongo = color_ck->data.c_str();
            ROS_WARN("My team is starting from [%s]!!!", color_ck->data.c_str());
            // game_side  = true;
        }
    }
}

// ROBOT EHLEH COMMAND HULEEH HESEG
void key_changed(const std_msgs::String::ConstPtr& msg)//this flag is start or stop 
{
    ROS_ERROR("[Admin] key pressed: [%s]", msg->data.c_str());
    if(msg->data == "s"){//start 
        ROS_INFO("[Admin] key pressed: [%s]", msg->data.c_str());
        start_cmd = true;
        game_status = 2;
    }
    if(msg->data == "r"){//retry
        ROS_INFO("[Admin] key pressed: [%s]", msg->data.c_str());
        start_cmd = true;
        game_status = 4; 
    }

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "MAIN_PROGRAMM");
    ROS_INFO("Main programm is working now!!!");
    ros::NodeHandle nh;
    // std::string ongo = "red";

    // ongo = nh.getParam("game_side", ongo);
    // nh.getParam("camera_point_x", camera_point_x);
    // nh.getParam("camera_point_y", camera_point_y);
    // ongo = "red";
     int counter = 1;
    
    ros::Subscriber  keychecker_sub = nh.subscribe("/key_checker", 10, &key_changed);//waiting starting commmand(from python code or STM32 IR sensor)
    ros::Subscriber back = nh.subscribe("/wall", 10, &wall_clk);//checking back side Start or Retry zone    
    ros::Subscriber sensor_status = nh.subscribe("/sensor", 100, &sensor_clk);
    ros::Subscriber imu_check = nh.subscribe("/imu", 10, &imu_callback);//Imu hazaih ued true false butsaana
    ros::Publisher speed = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
    ros::Subscriber camera = nh.subscribe("/camera_data", 10, &camera_detect);//from darknet 
    ros::Publisher brushless = nh.advertise<std_msgs::Int8>("/brushdc", 20);
    ros::Subscriber encoder = nh.subscribe("/encoder", 10, &encoder_call);
    ros::Publisher  dc_pos = nh.advertise<std_msgs::UInt16>("pos_cmd", 30);
    ros::Publisher res_encoder = nh.advertise<std_msgs::Int16>("coordinate", 10);
    
    silo_dist = (silo.back() - 1) * 75 + 50;
    silo.pop_back();

    ros::Rate loop_rate(200);  
    while(ros::ok()){
        // system("clear");
        // std::cout<<"Game status is "<<game_status<<std::endl;
        ROS_ERROR("Game status = [%d]", game_status);
        if(game_status < 5){
            if(game_status == 0){
                    if(ehlel == 1){
                        game_status = 1;
                        prev_status = 0;
                        velocity_publishing(0, 0, 0);
                        speed.publish(vel_pub);
                    }else if(ehlel == 2){
                        game_status = 3;
                        prev_status = 0;
                    }
            }else if(game_status == 1){
                
                if(start_cmd){ 
                    game_status = 2;
                }else{
                    system("clear");
                    std::cout<<"Game status is "<<game_status<<std::endl;
                    std::cout<<"My team is from: "<<ongo<<std::endl;
                    ROS_WARN_ONCE("The robot is wating start command!!!!");
                    std::cout<<"Robot is waiting start command: "<<std::endl;
                }

            }else if(game_status == 2){
                //start to retry distance
                ROS_WARN_ONCE("Robot is moving");
                int left_setpoint=20;
                int op=-1;
                if(ongo=="red"){
                    op=-1;
                    distance=front_distance;
                }
                else if(ongo=="blue"){
                    op=1;
                    distance=back_distance;
                }
                prev_status = 2;
                if(distance > 150 || distance==0){
                    std::cout<<"distance: "<<distance<<std::endl;
                    system("clear");
                    vel_pub.linear.x = PID(0.6, 0, 0.000, left_setpoint,left_distance, 127, 20);//int utga butsaana
                    vel_pub.linear.y =  op*127;
                    vel_pub.angular.z= (-1)*op*PID(1.6,  0.0, 0.000, 90, theta, 127, 20);//int utga butsaana
                    speed.publish(vel_pub);
                }else{
                    vel_pub.linear.x = 0;
                    vel_pub.linear.y = op*127;
                    vel_pub.angular.z = 0;
                    speed.publish(vel_pub);
                    sleep(2);
                    game_status = 4;
                }
            }else if(game_status == 3){
                if(prev_status == 2){
                    game_status = 4;
                }else if(prev_status == 0){vel_pub.angular.z= PID(0.6, 0.0, 0.000, 90, theta, 127, 20);//int utga butsaana
                    if(game_side == false){
                        system("clear");
                        std::cout<<"Game status is: "<< game_status<<std::endl;
                        std::cout<<"We are playing from: "<< ongo << std::endl;
                        ROS_ERROR("The ROBOT waiting start cmd!!!");
                        if(start_cmd){
                            game_status = 4;
                        }else{
                            velocity_publishing(0, 0, 0);
                            speed.publish(vel_pub);
                        }
                    }else{
                        ROS_WARN("We are playing from BLUE!!!");

                    }
                }
            }else if (game_status ==4){
                int front_setpoint=60;
                int left_setpoint=20;
                int op=1;
                if(ongo=="red"){
                    distance=left_distance;
                    op=1;
                    front_setpoint=70;
                }
                else if(ongo=="blue"){
                    distance=back_distance;
                    op=-1;
                    front_setpoint=60;
                }
                if(distance<180){
                    left_setpoint=400;
                }
                else{
                    left_setpoint=20;
                }
                    system("clear");
                    ROS_INFO("left distance = %d", left_distance);
                    ROS_INFO("front distance = %d", front_distance);
                    ROS_INFO("thete of left = %f", theta);
                    vel_pub.linear.x = PID(0.6, 0.0, 0.00, left_setpoint, left_distance, 127, 20);
                    vel_pub.linear.y = op*PID(0.7, 0.0, 0.00, front_setpoint, distance, 127, 20);
                    vel_pub.angular.z = op*PID(2, 0.0, 0.00, 90, theta, 127, 20);
                    speed.publish(vel_pub);
                    if(vel_pub.linear.x < 5 && vel_pub.linear.x > -5 && vel_pub.linear.y < 5 && vel_pub.linear.y > -5 && theta > 87 && theta < 93){
                        vel_pub.linear.x = 0;
                        vel_pub.linear.y = op*-127;
                        vel_pub.angular.z = 0;
                        speed.publish(vel_pub);
                        sleep(2);
                        if(ongo == "blue"){
                            //turn right 90 degree funcion
                            // if(
                        }
                        if(ongo == "red"){
                            //turn left 90 degree function
                            //if
                        }
                        reset_cordinate.data = 0;
                        res_encoder.publish(reset_cordinate);
                        game_status = 5;
                    }
                // }
            }
        }else{
            // ROS_WARN("ROBOT IS IN AREA 3!!!");

        // // ROBOT AREA 3 DEER GARAAD BOMBOGRUU HARNA
            if(game_status == 5){
                dcPos.data=0;
                dc_pos.publish(dcPos);
                // int op = 0;
                // if(ongo == "red"){
                //     op = -1;
                // }else if(ongo == "blue"){
                //     op = 1;
                // }
                if(robot_theta <= 85 || robot_theta >=92){
                    vel_pub.linear.x = 0;
                    vel_pub.linear.y = 0;
                    vel_pub.angular.z = -PID(1.5, 0, 0, 90, robot_theta, 127, 40);
                    speed.publish(vel_pub);

                }else{
                //90 gradus erge 
                //ergesen bol 
                    last_x = robot_x;
                    last_y = robot_y;
                    last_theta = robot_theta;
                    game_status = 6;
                }
            }
            else if(game_status == 6){
                system("clear");
                BLDC.data = 1;
                brushless.publish(BLDC);
                if(take_ball == true){//bomgog haragdah uyed 
                    // system("clear");
                    // ROS_INFO("W = %d", w);
                    
                    if(!((w <= camera_point_y + 5) && (u <= camera_point_x + 3) && (u >= camera_point_x - 3))){  

                        vel_pub.linear.y = PID(0.6, 0, 0, camera_point_y, w, 70 , 10);
                        vel_pub.linear.x = -PID(0.6, 0, 0, camera_point_x, u, 70, 10);       
                        vel_pub.angular.z = 0;
                        speed.publish(vel_pub);
                        ROS_INFO("nearest_ball : [%d]", nearest_ball);
                        ROS_INFO("X: - [%d]", u);
                        ROS_INFO("Y: - [%d]", w);
                        ROS_INFO("Min val: [%d]", min_val);

                    }else{
                        while(!robot_ball){

                            velocity_publishing(0,-50,0);
                            speed.publish(vel_pub);
                            ros::spinOnce();      

                        }

                        velocity_publishing(0,0,0);
                        speed.publish(vel_pub);

                    }
                    if(robot_ball){

                        game_status = 7;
                        prev_theta = robot_theta + 45;

                    }
                }else{
                    
                    velocity_publishing(0,0,0);
                    speed.publish(vel_pub);
                }
            }
            
            else if(game_status == 7){
                system("clear");
                ROS_INFO("last_x = [%d]", last_x);
                ROS_INFO("last_y = [%d]", last_y);
                if(true_ball != 0){// 0 = purple, 1 = red or blue/
                    //PID last pos
                    int diff_x = last_x - robot_x;
                    int diff_y = last_y - robot_y;
                    float diff_theta = atan2(diff_y, diff_x) - (robot_theta * 3.14/180);
                    int length_xy = sqrt(pow(diff_x, 2) + pow(diff_y, 2));
                    diff_x=cos(diff_theta)*length_xy;
                    diff_y=sin(diff_theta)*length_xy;
                    ROS_INFO("orson");
                    ROS_INFO("diff_x = [%d]", diff_x);
                    ROS_INFO("diff_y = [%d]", diff_y);  
                    
                    if((diff_x>3 || diff_x<-3) && (diff_y>3 || diff_y<-3)){
                        vel_pub.linear.x = -PID(1, 0, 0, 0, diff_x, 127, 10);
                        vel_pub.linear.y = PID(1, 0, 0, 0, diff_y, 127, 10);
                        vel_pub.angular.z = 0;
                        speed.publish(vel_pub);
                    }else{
                        prev_theta = robot_theta + 180;
                        game_status = 8;
                    }
                    
                    ROS_INFO("last_x =========== [%d]", last_x);
                    ROS_INFO("last_y =========== [%d]", last_y);
                    
                }else{
                    if(prev_theta >= robot_theta){
                        vel_pub.linear.x = 0;
                        vel_pub.linear.y = 0;
                        vel_pub.angular.z = -127;//-PID(1.2, 0, 0, prev_theta, robot_theta, 100);
                        speed.publish(vel_pub);
                    }else{
                        BLDC.data = 2;
                        brushless.publish(BLDC);
                        prev_theta-=45;
                        game_status = 15;
                    }
                }
            }
            
            else if(game_status == 8){
                system("clear");
                if(255 >= robot_theta || 280 <= robot_theta){
                    vel_pub.linear.x = 0;
                    vel_pub.linear.y = 0;
                    vel_pub.angular.z = -PID(0.6, 0, 0, 270, robot_theta, 127, 40);
                    speed.publish(vel_pub);
                }else{
                    // if(theta<88||theta>92){
                        // vel_pub.angular.z = PID(2, 0, 0, 90, theta, 127, 30);
                    // }else{
                
                        game_status = 16;
                    // }
                }                
            }
            
            else if(game_status == 9){
                system("clear");
                int op=0;
                if(ongo == "red"){
                    op = 1;
                    distance = left_distance + 20;
                }
                if(ongo == "blue"){
                    op = -1;
                    distance = right_distance - 20;
                }  
                if(!robot_ball){
                    
                    // if(distance <= 170 || distance >= 185){
                    //     vel_pub.linear.x = op*PID(0.6, 0, 0, last_x, robot_x, 127, 40);
                    // }else{
                    //     vel_pub.linear.x = 0;
                    // }
                    if(front_distance > 175){
                        game_status = 17;
                        
                    }else{
                       vel_pub.linear.y = PID(0.8, 0, 0, 175, front_distance, 127, 40);
                    }
                    vel_pub.angular.z = -PID(2.0, 0, 0, 90, theta, 127, 40);

                    speed.publish(vel_pub);
                }
            }
            else if(game_status == 15){
                if(prev_theta < robot_theta){
                        vel_pub.linear.x = 0;
                        vel_pub.linear.y = 0;
                        vel_pub.angular.z = 100;//-PID(0.6, 0, 0, prev_theta, robot_theta, 80);
                        speed.publish(vel_pub);
                }else{
                    game_status = 6;
                }
            }
            
            else if(game_status == 16){
                system("clear");
                int op=0;
                if(ongo == "red"){
                    op = 1;
                    distance = left_distance + 20;
                }
                if(ongo == "blue"){
                    op = -1;
                    distance = right_distance - 20;
                }                    
                if(theta<88||theta>92){
                    vel_pub.angular.z = PID(2, 0, 0, 90, theta, 127, 30);
                }else{
                    if(distance <= silo_dist + 1 && distance >= silo_dist - 1 && front_distance <= 34 )  {
                            BLDC.data = 2;
                            brushless.publish(BLDC);
                            velocity_publishing(0,0,0);
                            speed.publish(vel_pub);
                            reset_cordinate.data = 270;
                            res_encoder.publish(reset_cordinate);
                            // ros::spinOnce();
                            sleep(2);
                            game_status = 9;
                            

                    }else{
                        ROS_INFO("silo dist = [%d]", silo_dist);
                        ROS_INFO("left distance = [%d]", distance);

                        vel_pub.linear.x =  op*PID(1.2, 0, 0, silo_dist, distance, 110, 15);
                        vel_pub.linear.y = PID(0.8, 0, 0, 33, front_distance, 100, 0);
                        vel_pub.angular.z = -PID(2.3, 0, 0, 90, theta, 80, 0);
                        speed.publish(vel_pub);
                    }
                }

                // if(distance <= silo_dist + 1 && distance >= silo_dist - 1 && front_distance <= 34 && theta < 92 && theta > 90)  {
                //         BLDC.data = 2;
                //         brushless.publish(BLDC);
                //         velocity_publishing(0,0,0);
                //         speed.publish(vel_pub);
                //         sleep(1);
                //         game_status = 9;
                // }else{
                //     ROS_INFO("silo dist = [%d]", silo_dist);
                //     ROS_INFO("left distance = [%d]", distance);

                //     vel_pub.linear.x =  PID(1.3, 0, 0, silo_dist, distance, 127, 30);
                //     vel_pub.linear.y = PID(1.3, 0, 0, 33, front_distance, 127, 20);
                //     vel_pub.angular.z = PID(1.2, 0, 0, 90, theta, 127, 20);
                //     speed.publish(vel_pub);
                // }
            }
            
            else if(game_status == 17){
                
                if(robot_theta >= 92){

                    vel_pub.linear.x = 0;
                    vel_pub.linear.y = 0;
                    vel_pub.angular.z = -PID(1.5, 0, 0, 90, robot_theta, 127, 40);
                    speed.publish(vel_pub);

                }else{

                    // reset_cordinate.data = 90;
                    // res_encoder.publish(reset_cordinate);
                    silo_dist = (silo.back() - 1) * 75 + 50;
                    silo.pop_back();
                    last_theta = 90;
                    last_x = robot_x;
                    last_y = robot_y;
                    game_status = 6;

                }
            }
        }
         ros::spinOnce();
        loop_rate.sleep(); 
    }
    return 0;
}