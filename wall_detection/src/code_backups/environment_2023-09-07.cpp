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

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
// ------------------

// pcl::PointCloud<pcl::PointXYZI>::Ptr planeCloud(new pcl::PointCloud<pcl::PointXYZI>);

pcl::ModelCoefficients::Ptr plane_coeff(new pcl::ModelCoefficients);

std::ofstream plane_coef_File;
bool export_plane_coef = false;

std::ofstream objs_loc_File;
bool export_objs_loc = true;
// ------------------
// function takes currently fitted plane coefficient
// returns kalman filtered plane coefficients
// ------------------

// ------------------
// function gets point cloud and find best fitting plane based on ransac algorithm
// returns plane parameters such as a, b, c, and d in format of 'pcl::ModelCoefficients::Ptr'
// ------------------
pcl::ModelCoefficients::Ptr mGet_plane_coeff( pcl::PointCloud<pcl::PointXYZI>::Ptr &planeCloud )
{
    auto mstartTime = std::chrono::steady_clock::now();
    // *planeCloud = *segmentedPlane.second;
    pcl::ModelCoefficients::Ptr plane_coeff_temp(new pcl::ModelCoefficients);
    // std::cout << "fitting start " << planeCloud->size() << std::endl;
    pcl::SACSegmentation<pcl::PointXYZI> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(50);
    seg.setDistanceThreshold(0.10);

    pcl::PointIndices::Ptr sac_inlier(new pcl::PointIndices);
    seg.setInputCloud(planeCloud);
    seg.segment(*sac_inlier, *plane_coeff_temp);
    // std::cout << "plane coeffs are: a=" << plane_coeff->values[0] << ", b=" << plane_coeff->values[1] << ", c=" << plane_coeff->values[2] << ", d=" << plane_coeff->values[3] << std::endl;
    auto mendTime = std::chrono::steady_clock::now();
    auto melapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(mendTime - mstartTime);
    std::cout << "plane fitting took " << melapsedTime.count() << " milliseconds" << std::endl;

    return plane_coeff_temp;
}
// ------------------
// function gets plane parameters such as a, b, c, and d in format of 'pcl::ModelCoefficients::Ptr'
// returns ROTATION matrix and TANSLATION vector that could be used to convert
// points on plane(ground) to world(lidar) coordite system 
// ------------------
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
    Eigen::Vector3d plane_T_vect = plane_z*( abs(plane_coeff_t->values[3]/plane_normal.norm()));

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

    return std::make_pair(R_mat.transpose(), plane_T_vect);
}
// ------------------
// function gets plane parameters such as a, b, c, and d in format of 'pcl::ModelCoefficients::Ptr'
// returns quaternion between plane coordinate system and world coordinate system and Translation vector
// ------------------
std::pair<Eigen::Quaterniond, Eigen::Vector3d>  mGet_plane_qauternion(pcl::ModelCoefficients::Ptr &plane_coeff_t)
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

    Eigen::Vector3d plane_T_vect = plane_z*( abs(plane_coeff_t->values[3]/plane_normal.norm()));

    Eigen::Matrix3d plane_coord_matrix;
    plane_coord_matrix << plane_x,  plane_y,  plane_z; 

    Eigen::Quaterniond plane_quaternion(plane_coord_matrix);

    return std::make_pair(plane_quaternion, plane_T_vect);
}
// /*
// find 8 points of cluster in plane(ground) coordinate system
// */
template<typename PointT>
pcl::PointCloud<pcl::PointXYZ>::Ptr mGet_boundingBox_8pts(typename pcl::PointCloud<PointT>::Ptr cluster, Eigen::Matrix3d Rotation_matrix, Eigen::Vector3d Translation_vector)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cluster(new pcl::PointCloud<pcl::PointXYZI>);
    Eigen::Matrix4d tranformation_mat =  Eigen::Matrix4d::Identity();
    tranformation_mat.block<3,3>(0,0) = Rotation_matrix;
    tranformation_mat.block<3,1>(0,3) = Translation_vector;
    pcl::transformPointCloud(*cluster, *transformed_cluster, tranformation_mat);

    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);
    // Box box=pointProcessorI->BoundingBox(transformed_cluster);

    pcl::PointCloud<pcl::PointXYZ>::Ptr bb_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    Eigen::Vector3f bp1 = Eigen::Vector3f(minPoint.x, minPoint.y, minPoint.z);
    Eigen::Vector3f bp7 = Eigen::Vector3f(maxPoint.x, maxPoint.y, maxPoint.z);

    bb_cloud->push_back( pcl::PointXYZ( bp1(0), bp1(1), bp1(2)));
    bb_cloud->push_back( pcl::PointXYZ( bp7(0), bp1(1), bp1(2)));
    bb_cloud->push_back( pcl::PointXYZ( bp7(0), bp7(1), bp1(2)));
    bb_cloud->push_back( pcl::PointXYZ( bp1(0), bp7(1), bp1(2)));
    bb_cloud->push_back( pcl::PointXYZ( bp1(0), bp1(1), bp7(2)));
    bb_cloud->push_back( pcl::PointXYZ( bp7(0), bp1(1), bp7(2)));
    bb_cloud->push_back( pcl::PointXYZ( bp7(0), bp7(1), bp7(2)));
    bb_cloud->push_back( pcl::PointXYZ( bp1(0), bp7(1), bp7(2)));

    tranformation_mat.block<3,3>(0,0) = Rotation_matrix.inverse();
    tranformation_mat.block<3,1>(0,3) = -Translation_vector;

    pcl::PointCloud<pcl::PointXYZ>::Ptr bb_cloud_transformed(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::transformPointCloud(*bb_cloud, *bb_cloud_transformed, tranformation_mat);        
    return bb_cloud_transformed;
}
// ----------------------------------------------------------------

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}

// ++ ------------
Eigen::Matrix4f kf_A = Eigen::Matrix4f::Identity();
Eigen::Matrix4f kf_H = Eigen::Matrix4f::Identity();
Eigen::Matrix4f kf_Q = Eigen::Matrix4f::Identity()*0.1;
Eigen::Matrix4f kf_R = Eigen::Matrix4f::Identity()*0.5;

Eigen::Vector4f kf_X; // << -0.0171553, 0.0204001, 0.999645, 1.41017;
Eigen::Matrix4f kf_P = Eigen::Matrix4f::Identity();
bool kf_initiated = false;

Eigen::Vector4f kf_X_pred;
Eigen::Matrix4f kf_P_pred;

Eigen::Vector4f kf_X_meas;

Eigen::Matrix4f kf_K;

// pcl::ModelCoefficients::Ptr plane_coeff_for_kf(new pcl::ModelCoefficients);

bool FIRST_PLANE_FITTED = false;

// ++ ------------

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud)
{
  // ----------------------------------------------------
  // -----Open 3D viewer and display City Block     -----
  // ----------------------------------------------------

  //ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
  //pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
//   inputCloud = pointProcessorI->FilterCloud(inputCloud, 0.3, Eigen::Vector4f (-40, -10, -2, 1), Eigen::Vector4f ( 50, 20, 2, 1));
    inputCloud = pointProcessorI->FilterCloud(inputCloud, 0.15, Eigen::Vector4f (0, -40, -2, 1), Eigen::Vector4f (  100, 40, 5, 1));

    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr,pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentedPlane;
    // segmentedPlane = pointProcessorI->RANSAC_PlaneSegment_Scratch(inputCloud,50,0.15);
    if(FIRST_PLANE_FITTED == false){
         segmentedPlane = pointProcessorI->RANSAC_PlaneSegment_Scratch(inputCloud,100,0.15);
         FIRST_PLANE_FITTED = true;
    }
    else {
        segmentedPlane = pointProcessorI->RANSAC_PlaneSegment_with_init_Scratch(inputCloud, plane_coeff, 20,0.30);
    }

    // ----------------------
    plane_coeff = mGet_plane_coeff( segmentedPlane.second );
    // plane_coef_File << plane_coeff->values[0] << ", " << plane_coeff->values[1] << ", " << plane_coeff->values[2] << ", " << plane_coeff->values[3] << std::endl;
    kf_X_meas << plane_coeff->values[0], plane_coeff->values[1], plane_coeff->values[2], plane_coeff->values[3];  
    // ++ ------------
    if (kf_initiated==false){
        kf_X << kf_X_meas; 
        std::cout << "plane coeff initiated as: " << kf_X << std::endl;
        kf_initiated=true;
    }
    else
    {
        // prediction
        kf_X_pred = kf_A*kf_X;
        kf_P_pred = kf_A*kf_P*kf_A.transpose() + kf_Q;
        //update
        Eigen::Matrix4f denom = kf_H * kf_P_pred * kf_H.transpose() + kf_R;
        kf_K = ( kf_P_pred*kf_H.transpose() )*denom.inverse();
        kf_X = kf_X_pred + kf_K * ( kf_X_meas - kf_H*kf_X_pred );
        kf_P = (Eigen::Matrix4f::Identity() - kf_K * kf_H)*kf_P_pred;
        plane_coeff->values[0] = kf_X(0);
        plane_coeff->values[1] = kf_X(1);
        plane_coeff->values[2] = kf_X(2);
        plane_coeff->values[3] = kf_X(3);
    }
    if(export_plane_coef)
        plane_coef_File << plane_coeff->values[0] << ", " << plane_coeff->values[1] << ", " << plane_coeff->values[2] << ", " << plane_coeff->values[3] << std::endl;

    // ++ ------------
    std::pair<Eigen::Matrix3d, Eigen::Vector3d> plane_RT = mGet_plane_Rot_Tran(plane_coeff);
    // ----------------------
    std::pair<Eigen::Quaterniond, Eigen::Vector3d> plane_Quaternion = mGet_plane_qauternion(plane_coeff);
    Eigen::Quaternionf plane_f_quaternion = plane_Quaternion.first.cast<float>();
    Eigen::Vector3f plane_t_vector = -plane_Quaternion.second.cast<float>();
    // std::cout << "quaternion is: " << plane_f_quaternion.coeffs() << std::endl;
    // ----------------------
        
    renderPointCloud(viewer, segmentedPlane.first,"Obstacle Cloud",Color(0,0,1));
    renderPointCloud(viewer, segmentedPlane.second,"Plane Cloud",Color(1,0,1));
    
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->EuclideanCluster_scratch_clustering(segmentedPlane.first, 0.5, 50, 5000);
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};
    
    std::chrono::milliseconds melapsedTime(0);
    // /*
    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        // std::cout << "cluster size ";
        // pointProcessorI->numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId%3]);
        
        // ----------------------
        auto mstartTime = std::chrono::steady_clock::now();

        pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cluster(new pcl::PointCloud<pcl::PointXYZI>);
        Eigen::Matrix4d tranformation_mat =  Eigen::Matrix4d::Identity();
        tranformation_mat.block<3,3>(0,0) = plane_RT.first;
        tranformation_mat.block<3,1>(0,3) = plane_RT.second;
        pcl::transformPointCloud(*cluster, *transformed_cluster, tranformation_mat);
        Box box=pointProcessorI->BoundingBox(transformed_cluster);     

        Box_params box_prms=pointProcessorI->BoundingBox_params(transformed_cluster);

        pcl::PointXYZI minPoint, maxPoint;
        pcl::getMinMax3D(*cluster, minPoint, maxPoint);        
        box_prms.center = Eigen::Vector3f( 0.5*(maxPoint.x+minPoint.x), 0.5*(maxPoint.y+minPoint.y), 0.5*(maxPoint.z+minPoint.z) );
        box_prms.quaternion = plane_f_quaternion;

        auto mendTime = std::chrono::steady_clock::now();
        melapsedTime = melapsedTime + std::chrono::duration_cast<std::chrono::milliseconds>(mendTime - mstartTime);
        // // ------------------------
        
        if(     (0.8 < box_prms.height && box_prms.height < 3.5) 
            &&  ( 0.5 < box_prms.width && box_prms.width < 5.0)
            &&  ( box_prms.lenght < 5.0) 
            &&  ( 1 > box_prms.dist_to_road )
            &&  ( box_prms.center.norm() < 70 ) ) // check object size and distance
        {
            Color mcol( 0.8, 0.8, 0.0);
            renderBox_params(viewer, box_prms, clusterId, mcol, 0.3);
            // std::cout << "obj dist: "<< box_prms.center.norm() << std::endl;
            if(export_objs_loc)
                objs_loc_File << box_prms.center[0] << ", " << box_prms.center[1] << ", " ;
        }
        // else
        //     if(     0.5 < box_prms.height 
        //              && 1 > box_prms.lenght
        //              && 1 > box_prms.width
        //              && 1.5 < (box_prms.height/box_prms.lenght) 
        //              && 1.5 < (box_prms.height/box_prms.width) 
        //              &&  ( box_prms.center.norm() < 50 ) )
        //     {
        //         Color mcol( 0.0, 0.8, 0.8);
        //         float obj_transfarancy = (box_prms.height/box_prms.lenght) * (box_prms.height/box_prms.width) ;
        //         renderBox_params(viewer, box_prms, clusterId, mcol, obj_transfarancy/10);
        //         // std::cout << "obj dist: "<< box_prms.center.norm() << std::endl;
        //     }
        //     else
        //     {
        //         Color mcol( 0.8, 0.6, 0.8);
        //         renderBox_params(viewer, box_prms, clusterId, mcol,  0.05);
        //     }
        ++clusterId;
    }
    if(export_objs_loc)
        objs_loc_File << std::endl;
    // */
    std::cout << "cluster rotation took " << melapsedTime.count() << " milliseconds" << std::endl;
//renderPointCloud(viewer,inputCloud,"inputCloud");
}

void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = true;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    //Create lidar sensor 
    Lidar* lidar = new Lidar(cars,0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud= lidar->scan();
    
    //Point cloud data viewer
    //renderPointCloud(viewer, inputCloud,"Input Cloud",color);
    
    //lidar Rays Viewer
    //renderRays(viewer, lidar->position, inputCloud);
    
    //Create point processor
    ProcessPointClouds<pcl::PointXYZ>* pointProcess= new ProcessPointClouds<pcl::PointXYZ>();
    //count number of clouds
    pointProcess->numPoints(inputCloud);
    //Segmenting the plane
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr,pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentedPlane = pointProcess->SegmentPlane(inputCloud,100,0.1);

    renderPointCloud(viewer, segmentedPlane.first,"Obstacle Cloud",Color(0,0,1));
    renderPointCloud(viewer, segmentedPlane.second,"Plane Cloud",Color(1,1,1));

    //Cluster objects based on Obstacle cloud and Plane cloud
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcess->Clustering(segmentedPlane.first, 2, 4, 50);
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};

    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        // std::cout << "cluster size ";
        // pointProcess->numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);
        Box box=pointProcess->BoundingBox(cluster);
        renderBox(viewer,box,clusterId);
        ++clusterId;
    }
    

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

int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    // -----------+++++++++++
    if(export_plane_coef){
        // plane_coef_File = std::ofstream("/home/reamond/Documents/plane_coef_ransac_with_init.txt");
        plane_coef_File = std::ofstream("/home/reamond/Documents/plane_coef_inited_ransac_kf.txt");
        if(!plane_coef_File){
            std::cerr << "Error opening file 'plane_coef.txt' for writing!" << std::endl;
            return 1;
        }
    }
    // -----------+++++++++++
    if(export_objs_loc){
        objs_loc_File = std::ofstream("/home/reamond/Documents/objs_location.txt");
        if(!objs_loc_File){
            std::cerr << "Error opening file 'objs_location.txt' for writing!" << std::endl;
            return 1;
        }
    }
    // -----------+++++++++++

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    // CameraAngle setAngle = XY;
    // CameraAngle setAngle = BehindCar;
    CameraAngle setAngle = FromCamParamFile;

    initCamera(setAngle, viewer);
    //simpleHighway(viewer);
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    // -----------------------------------------------------------------------
    // std::string gen_Path = "../src/sensors/data/pcd/data_2";
    // std::string gen_Path = "/home/reamond/Documents/pcd_data_2023-07-10";
    // std::string gen_Path = "/home/reamond/Documents/data_acq_2023-07-29";
    std::string gen_Path = "/home/reamond/Documents/data_acq_2023-08-09";
    // std::string gen_Path = "/home/reamond/Documents/vc_code_projects/Unscented-Kalman-Filter-with-LiDAR-and-Radar-master/src/sensors/data";
    // int start_frame = 2400;
    int start_frame = 30;
    static const bool with_image = true;
    // static const bool with_image = false;
    // -----------------------------------------------------------------------
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd(gen_Path+"/pcd");
    auto streamIterator = stream.begin();
    streamIterator+=start_frame;
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;
    
    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    std::string image_dataPath = gen_Path+"/image";
    std::vector<boost::filesystem::path> image_paths(boost::filesystem::directory_iterator{image_dataPath}, boost::filesystem::directory_iterator{});
    // sort files in accending order so playback is chronological
    sort(image_paths.begin(), image_paths.end());

    auto image_streamIterator = image_paths.begin();
    image_streamIterator+=start_frame;
    static const std::string OPENCV_WINDOW = "image window";
    if(with_image)
        cv::namedWindow(OPENCV_WINDOW);

    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
    while (!viewer->wasStopped ())
    {
          // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // -----------------
        viewer->addPlane(*plane_coeff, "plane");
        // -----------------

        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        std::cout << "stream str: " << (*streamIterator).string() << std::endl;
        cityBlock(viewer, pointProcessorI, inputCloudI);

        streamIterator++;
        if(streamIterator == stream.end()){
            // streamIterator = stream.begin();             // <---------------------------- stream start from the begining
            break;
        }

        if(with_image){
            std::string image_name = (*image_streamIterator).string();
            cv::Mat image = cv::imread(image_name);            
            if (!image.empty()) {
                cv::imshow(OPENCV_WINDOW, image);
                cv::waitKey(1);
            }
            else
            {
                std::cerr << "Error: Failed to open or read the image from: " << image_name << std::endl;
                return -1;
            }

            image_streamIterator++;
            if(image_streamIterator == image_paths.end())
                image_streamIterator = image_paths.begin();
        }

        viewer->spinOnce();
        // viewer->saveCameraParameters("cameraParameter.txt");
    }

    if(with_image)
        cv::destroyWindow(OPENCV_WINDOW);
    if(export_plane_coef)        
        if(plane_coef_File)
            plane_coef_File.close();
    if(export_objs_loc)        
        if(objs_loc_File)
            objs_loc_File.close();            
}
