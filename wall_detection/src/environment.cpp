/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

// ------------------
#include "pcl/sample_consensus/ransac.h"
#include "pcl/sample_consensus/sac_model_plane.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/common/transforms.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/concatenate.h>
#include <pcl/common/transforms.h>


// #include <opencv2/imgproc.hpp>
// #include <opencv2/highgui.hpp>
#include <pcl/sample_consensus/sac_model_line.h>
#include <vector>  
#include <math.h>


#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Int16MultiArray.h>
//#include <laser_geometry/laser_geometry.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>  
#include <visualization_msgs/Marker.h>
#include <math.h>
// ------------------
using namespace std::chrono_literals;

// ------------------

// ------------------

// ------------------

// pcl::PointCloud<pcl::PointXYZI>::Ptr planeCloud(new pcl::PointCloud<pcl::PointXYZI>);



// ------------------
// function takes currently fitted plane coefficient
// returns kalman filtered plane coefficients
// ------------------

// ------------------
// function gets point cloud and find best fitting plane based on ransac algorithm
// returns plane parameters such as a, b, c, and d in format of 'pcl::ModelCoefficients::Ptr'
// ------------------

pcl::visualization::PCLVisualizer::Ptr
simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  //viewer->addCoordinateSystem (1.0, "global");
  viewer->initCameraParameters ();
  return (viewer);
}
// ------------------
// function gets plane parameters such as a, b, c, and d in format of 'pcl::ModelCoefficients::Ptr'
// returns ROTATION matrix and TANSLATION vector that could be used to convert
// points on plane(ground) to world(lidar) coordite system 
// ------------------

// ------------------
// function gets plane parameters such as a, b, c, and d in format of 'pcl::ModelCoefficients::Ptr'
// returns quaternion between plane coordinate system and world coordinate system and Translation vector
// ------------------

// /*
// find 8 points of cluster in plane(ground) coordinate system
// */

// ----------------------------------------------------------------


// ++ ------------


ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
 ros::Publisher wall_data;
 ros::Publisher seg_points;
// pcl::ModelCoefficients::Ptr plane_coeff_for_kf(new pcl::ModelCoefficients);


// ++ ------------

// void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud)
void wallBlock( pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud)
{
  // ----------------------------------------------------
  // -----Open 3D viewer and display City Block     -----
  // ----------------------------------------------------

  //ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
  //pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
    inputCloud = pointProcessorI->FilterCloud(inputCloud, 0.001, Eigen::Vector4f (-6.2, -6.2, -0.2, 1), Eigen::Vector4f ( 6.2, 6.2, 0.2, 1));
    // inputCloud = pointProcessorI->FilterCloud(inputCloud, 0.15, Eigen::Vector4f (-, -40, -2, 1), Eigen::Vector4f (  100, 40, 5, 1));

    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr,pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentedPlane;
    double theta = -M_PI / 2; // -90 degrees

    // Define the transformation matrix
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ()));
    pcl::transformPointCloud(*inputCloud, *inputCloud, transform);

    segmentedPlane.first=inputCloud;

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->EuclideanCluster_scratch_clustering(segmentedPlane.first, 0.15, 25, 500);
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};
    //std::cout << "cb-1" << std::endl;
    
    std::chrono::milliseconds melapsedTime(0);
    std::chrono::milliseconds m_renderTime(0);
    int i=0;
    renderPointCloud(viewer, segmentedPlane.first,"Obstacle",Color(0,1,0));
    pcl::PointXYZ vector_point[10];
    
       pcl::PointCloud<pcl::PointXYZI>::Ptr cloudall (new pcl::PointCloud<pcl::PointXYZI>);
    // /*
  for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        //pcl::concatenatePointCloud(*cloud, *cluster, *cloud);
        std::cout << "cluster size ";
        pointProcessorI->numPoints(cluster);
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
        //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud (*cluster, *cloud);
        cloudall->points.reserve(cloudall->size() + cluster->size());
        *cloudall += *cloud;
        //pcl::concatenatePointCloud(*cloud, *cloud2, *cloudall);
    }
    renderPointCloud(viewer, cloudall,"Obstacle Cloud",Color(0,0,1));

     Eigen::VectorXf coefficients;
    line_data line_o[10];
    double angle_x;
    int line_count=0;
    inputCloud->clear();
    while(cloudall->size()>30){
         line_data line_coefficients;
         std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr,pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentedLine;
         std::pair<line_data,std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr,pcl::PointCloud<pcl::PointXYZI>::Ptr>> allResult;
        //  std::cout<<"8r mur"<<std::endl;
         allResult = pointProcessorI->RANSAC_LineSegment_Scratch(cloudall,200,0.02);
         line_coefficients=allResult.first;
         segmentedLine=allResult.second;
        //  std::cout<<"9r mur"<<std::endl;
         std::cout<<"line coefficients " << line_coefficients.a << ", "<< line_coefficients.b << ", "<< line_coefficients.c << std::endl;
         if(segmentedLine.second->size()<30){
            break;
         }
        line_count++;
        char c = static_cast<char>(line_count);
        renderPointCloud(viewer, segmentedLine.second, &c, Color(1, 0, 0));
        //inputCloud->clear();  
        *inputCloud+=*segmentedLine.second;
        cloudall=segmentedLine.first;
        std::cout<<"point size "<<cloudall->size()<<std::endl;
 
        std::cout<<"angle="<<line_coefficients.angle_x<<" degree"<<std::endl;

        line_o[i]=line_coefficients;
        i++;

        c = static_cast<char>(line_count+20);
        viewer->addLine(line_coefficients.p1,line_coefficients.p2 , 1.0, 0.0, 0.0, &c);
  
    }

    sensor_msgs::PointCloud2 line_seg;
    line_seg.header.frame_id="laser";
    pcl::toROSMsg(*inputCloud, line_seg);
    seg_points.publish(line_seg);
    int left_index=-1;
    int right_index=-1;
    int front_index=-1;
    int back_index =-1;
    std::cout<<line_count<< " line detected"<<std::endl;
    for(int i=0; i<line_count; i++){
        std::cout<<i<< "-r line is " << line_o[i].line_stat << " stat" <<std::endl;
        switch (line_o[i].line_stat)
        {
        case 1:
            if(front_index==-1){
                front_index=i;
            }
            break;
        case 2:
            if(left_index==-1){
                left_index=i;
            }
            break;
        case 3:
            if(back_index==-1){
                back_index=i;
            }
            break;
        case 4:
            if(right_index==-1){
                right_index=i;
            }
            break;
        default:
            break;
        }
    }



    std_msgs::Int16MultiArray coordinate_buff;
    // int coordinate_stat[4];

    pcl::PointXYZI inter_point;
    //line_data left_line;
    if(left_index!=-1){
        inter_point.x = (line_o[left_index].r-line_o[front_index].r)/(line_o[front_index].m-line_o[left_index].m);
        inter_point.y = (line_o[left_index].l-line_o[front_index].l)/(line_o[front_index].k-line_o[left_index].k);
        inter_point.z = 0;
        // coordinate_stat[0]=line_o[left_index].line_dist*100;
        coordinate_buff.data.push_back(line_o[left_index].line_dist*100);
        // coordinate_buff.data.push_back(line_o[front_index].line_dist*100);
        if(right_index==-1){
            coordinate_buff.data.push_back((5.875-line_o[left_index].line_dist)*100);
        }
        else{
            coordinate_buff.data.push_back(line_o[right_index].line_dist*100);
        }
        coordinate_buff.data.push_back(line_o[left_index].angle_x);
        // coordinate_stat[1]=line_o[front_index].line_dist*100;
        // coordinate_stat[2]=line_o[left_index].angle_x;
    }
    else{

        inter_point.x = (line_o[right_index].r-line_o[front_index].r)/(line_o[front_index].m-line_o[right_index].m)+cos((line_o[right_index].angle_x+90)*3.14/180)*5.875;
        inter_point.y = (line_o[right_index].l-line_o[front_index].l)/(line_o[front_index].k-line_o[right_index].k)+sin((line_o[right_index].angle_x+90)*3.14/180)*5.875;
        inter_point.z = 0;
        coordinate_buff.data.push_back((5.875-line_o[right_index].line_dist)*100);
        // coordinate_buff.data.push_back(line_o[front_index].line_dist*100);
        coordinate_buff.data.push_back(line_o[right_index].line_dist*100);
        coordinate_buff.data.push_back(line_o[right_index].angle_x);
        
        // coordinate_stat[0]=(5.875-line_o[right_index].line_dist)*100;
        // coordinate_stat[1]=line_o[front_index].line_dist*100;
        // coordinate_stat[2]=line_o[right_index].angle_x;
    }
    if(front_index!=-1){
        coordinate_buff.data.push_back(line_o[front_index].line_dist*100);

    }
    else{
        coordinate_buff.data.push_back(0);
    }
    if(back_index!=-1){
        // coordinate_stat[3]=line_o[back_index].line_dist*100;
        coordinate_buff.data.push_back(line_o[back_index].line_dist*100);
    }
    else{
        coordinate_buff.data.push_back(0);
    }
    wall_data.publish(coordinate_buff);
   // intersection->push_back(inter_point);
    viewer->removeCoordinateSystem("rr", 0);
    viewer->addCoordinateSystem (1, inter_point.x, inter_point.y, inter_point.z,  "rr",  0);

}


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0.2, 0.2, 0.2);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1); break;
        case FromCamParamFile : viewer->loadCameraParameters("cameraParameter.txt"); break;
        case BehindCar : viewer->setCameraPosition(-10, 0, 15, 1, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}





void point_cb(const sensor_msgs::PointCloud2::ConstPtr& input){
    sensor_msgs::PointCloud2 filtered_scan = *input;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(filtered_scan, *cloud);
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();
    wallBlock(cloud);
}

int main (int argc, char** argv)
{
    ros::init(argc, argv, "wall_follower");
    ros::NodeHandle nh;
    ros::Subscriber point_ros;
    // ros::Publisher wall_data;
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    //ros::Publisher marker_pub;
    point_ros=nh.subscribe<sensor_msgs::PointCloud2>("/scan/points", 10, point_cb);
    wall_data=nh.advertise<std_msgs::Int16MultiArray>("wall", 10);
    seg_points=nh.advertise<sensor_msgs::PointCloud2>("segmented_points",10);
    CameraAngle setAngle = FromCamParamFile;
    initCamera(setAngle, viewer);

     ros::spin();
 
}
