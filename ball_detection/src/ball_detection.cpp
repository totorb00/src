/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

// #include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

// ------------------
#include "pcl/sample_consensus/ransac.h"
#include "pcl/sample_consensus/sac_model_plane.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/common/transforms.h"

// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
// #include <pcl/common/eigen.h>
// #include <pcl/common/centroid.h>


#include <vector>



#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Int16MultiArray.h>
//#include <laser_geometry/laser_geometry.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>  
#include <visualization_msgs/Marker.h>
#include <math.h>


pcl::ModelCoefficients::Ptr plane_coeff(new pcl::ModelCoefficients);


ProcessPointClouds<pcl::PointXYZ>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZ>();
// pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));

ros::Publisher ball_points;
ros::Publisher plane_points;
ros::Publisher ball_center_points;


bool FIRST_PLANE_FITTED = false;

std::pair<Eigen::Matrix3d, Eigen::Vector3d> mGet_plane_Rot_Tran(pcl::ModelCoefficients::Ptr &plane_coeff_t)
{
    Eigen::Vector3d plane_normal(plane_coeff_t->values[0], plane_coeff_t->values[1], plane_coeff_t->values[2]);
    Eigen::Vector3d plane_x, plane_y, plane_z;
    Eigen::Vector3d temp_y(0, 1, 0), temp_x;
    plane_z = plane_normal/plane_normal.norm();
    temp_x = temp_y.cross(plane_z);
    plane_y = plane_z.cross(temp_x);
    plane_y = plane_y/plane_y.norm();

    plane_x = plane_y.cross(plane_z);
    plane_x = plane_x/plane_x.norm();
    Eigen::Vector3d plane_T_vect = plane_z*(plane_coeff_t->values[3]/plane_normal.norm());

    // ground_plane_center = pcl::PointXYZ(plane_T_vect(0), plane_T_vect(1), plane_T_vect(2) );

    double theta = acos( plane_z(2));
    double phi2  = atan2( plane_x(2), plane_y(2));
    double phi1  = atan2( plane_z(0), -plane_z(1));

    Eigen::Matrix3d R_mat;

    R_mat(0, 0) = cos(phi1)*cos(phi2) - cos(theta)*sin(phi2)*sin(phi1);
	R_mat(0, 1) = -cos(phi1)*sin(phi2) - cos(theta)*cos(phi2)*sin(phi1);
	R_mat(0, 2) = sin(phi1)*sin(theta);
	R_mat(1, 0) = sin(phi1)*cos(phi2) + cos(theta)*sin(phi2)*cos(phi1);
	R_mat(1, 1) = -sin(phi1)*sin(phi2) + cos(theta)*cos(phi2)*cos(phi1);
	R_mat(1, 2) = -cos(phi1)*sin(theta);
	R_mat(2, 0) = sin(theta)*sin(phi2);
	R_mat(2, 1) = sin(theta)*cos(phi2);
	R_mat(2, 2) = cos(theta);
    // plane_T_vect(0)=0;
    // plane_T_vect(1)=0;
    // plane_T_vect(2)=0;
    return std::make_pair(R_mat.transpose(), plane_T_vect);
}

// ++ ------------

void sphereBlock( pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud)
{
    inputCloud = pointProcessorI->FilterCloud(inputCloud, 0.02, Eigen::Vector4f (-5, -5, -5, 1), Eigen::Vector4f ( 5, 5, 5, 1));
    // inputCloud = pointProcessorI->FilterCloud(inputCloud, 0.15, Eigen::Vector4f (-, -40, -2, 1), Eigen::Vector4f (  100, 40, 5, 1));

    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr,pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentedPlane;
    std::pair<pcl::ModelCoefficients::Ptr,  std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr,pcl::PointCloud<pcl::PointXYZ>::Ptr>> allPlane;
    if(FIRST_PLANE_FITTED == false){
         allPlane = pointProcessorI->RANSAC_PlaneSegment_Scratch(inputCloud,50,0.02);
         segmentedPlane=allPlane.second;
         FIRST_PLANE_FITTED = true;
    }
    else {
        allPlane = pointProcessorI->RANSAC_PlaneSegment_with_init_Scratch(inputCloud, plane_coeff, 10, 0.02);
        segmentedPlane=allPlane.second;
    }
    // ++ ------------
    // ----------------------
    plane_coeff = allPlane.first;


    if(plane_coeff->values[2]>0){
        plane_coeff->values[0]*=(-1);
        plane_coeff->values[1]*=(-1);
        plane_coeff->values[2]*=(-1);
        plane_coeff->values[3]*=(-1);
    }

    std::pair<Eigen::Matrix3d, Eigen::Vector3d> plane_RT = mGet_plane_Rot_Tran(plane_coeff);



    

    // ----------------------
    //  std::cout << "8-r mur" << std::endl;   
    // renderPointCloud(viewer, segmentedPlane.first,"Obstacle Cloud",Color(0,0,1));
    // renderPointCloud(viewer, segmentedPlane.second,"Plane Cloud",Color(1,0,1));
    // std::cout << "9-r mur" << std::endl;   
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessorI->EuclideanCluster_scratch_clustering(segmentedPlane.first, 0.02, 50, 250);
    // std::cout << "10-r mur" << std::endl;   
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};
    
    std::chrono::milliseconds melapsedTime(0);
    std::chrono::milliseconds m_renderTime(0);
    // /*
    int i=0;
    pcl::PointCloud<pcl::PointXYZ>::Ptr centerCloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr ballCloud (new pcl::PointCloud<pcl::PointXYZ>);
    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        // std::cout << "cluster size ";
        // pointProcessorI->numPoints(cluster);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud (*cluster, *cloud);
           while(cloud->size()>20){
                //line_data line_coefficients;
                std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr,pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentedSphere;
                std::pair<pcl::PointXYZ, std::pair<typename pcl::PointCloud<pcl::PointXYZ>::Ptr, typename pcl::PointCloud<pcl::PointXYZ>::Ptr>> allResult;
                pcl::PointXYZ pR;
                //  std::cout<<"9r mur"<<std::endl;
                
                allResult=pointProcessorI->RANSAC_SphereSegment_Scratch(cloud,50, 0.008, 0.09);
                segmentedSphere=allResult.second;
                pR=allResult.first;
                if(segmentedSphere.second->size()<20){
                    break;
                }
                

                // ----------------------------------------------------------------------
                    // Compute centroid    // Compute centroid
                Eigen::Vector4f centroid;
                pcl::compute3DCentroid(*segmentedSphere.second, centroid);

                // Compute covariance matrix
                Eigen::Matrix3f covariance_matrix;
                pcl::computeCovarianceMatrixNormalized(*segmentedSphere.second, centroid, covariance_matrix);

                // Perform eigen decomposition
                Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance_matrix);
                Eigen::Matrix3f eigenvectors = eigen_solver.eigenvectors();
                Eigen::Vector3f eigenvalues = eigen_solver.eigenvalues();
                cout << "eigen values: " << eigenvalues.transpose() << endl; 

               if (eigenvalues(0) > 0.00015){
                    char c = static_cast<char>(i);
                    // renderPointCloud(viewer, segmentedSphere.second, &c, Color(1, 0, 0));
                    if(pR.x!=0 && pR.y!=0 && pR.z!=0){
                        centerCloud->points.push_back(pR);
                    }
                    *ballCloud += *segmentedSphere.second;
               }
                cloud=segmentedSphere.first;
                std::cout<<"point size "<<cloud->size()<<std::endl;
        

                i++;

        
            }
        // ----------------------

        ++clusterId;
    }
    Eigen::Matrix4d tranformation_mat =  Eigen::Matrix4d::Identity();
    Eigen::Vector3d vector_to_add(0, 0, 0);
    tranformation_mat.block<3,3>(0,0) = plane_RT.first;
    tranformation_mat.block<3,1>(0,3) = vector_to_add;
    for (size_t i = 0; i < segmentedPlane.second->points.size(); ++i)
    {
        segmentedPlane.second->points[i].x += plane_RT.second[0];
        segmentedPlane.second->points[i].y += plane_RT.second[1];
        segmentedPlane.second->points[i].z += plane_RT.second[2];
    }
    for (size_t i = 0; i < ballCloud->points.size(); ++i)
    {
        ballCloud->points[i].x += plane_RT.second[0];
        ballCloud->points[i].y += plane_RT.second[1];
        ballCloud->points[i].z += plane_RT.second[2];
    }
    for (size_t i = 0; i < centerCloud->points.size(); ++i)
    {
        centerCloud->points[i].x += plane_RT.second[0];
        centerCloud->points[i].y += plane_RT.second[1];
        centerCloud->points[i].z += plane_RT.second[2];
    }
    pcl::transformPointCloud(*segmentedPlane.second, *segmentedPlane.second, tranformation_mat);
    sensor_msgs::PointCloud2 plane_p;
    pcl::toROSMsg(*segmentedPlane.second, plane_p);
    plane_p.header.stamp = ros::Time::now();
    plane_p.header.frame_id="map";
    plane_points.publish(plane_p);
    sensor_msgs::PointCloud2 ball;
    // ball.header.stamp = ros::Time::now();
    // pcl_conversions::toPCL(ros::Time::now(), ball.header.stamp);
    pcl::transformPointCloud(*ballCloud, *ballCloud, tranformation_mat);
    pcl::toROSMsg(*ballCloud, ball);
    ball.header.frame_id="map";
    ball_points.publish(ball);
    sensor_msgs::PointCloud2 ball_c;
    pcl::transformPointCloud(*centerCloud, *centerCloud, tranformation_mat);
    pcl::toROSMsg(*centerCloud, ball_c);
    ball_c.header.stamp = ros::Time::now();
    ball_c.header.frame_id="map";  
    ball_center_points.publish(ball_c);
    // renderPointCloud(viewer, centerCloud, "center", Color(1, 1,1));
    // */
    std::cout << "cluster rotation took " << melapsedTime.count() << " milliseconds" << std::endl;
    std::cout << "Rendering rotation took " << melapsedTime.count() << " milliseconds" << std::endl;
}


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
// void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
// {

//     viewer->setBackgroundColor (0.2, 0.2, 0.2);
    
//     // set camera position and angle
//     viewer->initCameraParameters();
//     // distance away in meters
//     int distance = 16;
    
//     switch(setAngle)
//     {
//         case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
//         case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
//         case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
//         case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1); break;
//         case FromCamParamFile : viewer->loadCameraParameters("cameraParameter.txt"); break;
//         case BehindCar : viewer->setCameraPosition(-10, 0, 15, 1, 0, 1);
//     }

//     if(setAngle!=FPS)
//         viewer->addCoordinateSystem (1.0);
// }

void point_cb(const sensor_msgs::PointCloud2::ConstPtr& input){
    sensor_msgs::PointCloud2 filtered_scan = *input;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(filtered_scan, *cloud);
    sphereBlock( cloud);
}


int main (int argc, char** argv)
{
    ros::init(argc, argv, "wall_follower");
    ros::NodeHandle nh;
    ros::Subscriber point_ros;
    // ros::Publisher wall_data;
    // ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    //ros::Publisher marker_pub;
    point_ros=nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth/points", 10, point_cb);
    ball_points=nh.advertise<sensor_msgs::PointCloud2>("ball_points",10);
    ball_center_points=nh.advertise<sensor_msgs::PointCloud2>("ball_center",10);
    plane_points=nh.advertise<sensor_msgs::PointCloud2>("plane_points", 10);
    // CameraAngle setAngle = FromCamParamFile;


     ros::spin();           
}
