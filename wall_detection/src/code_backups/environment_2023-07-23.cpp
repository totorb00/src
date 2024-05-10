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
// pcl::PointCloud<pcl::PointXYZI>::Ptr planeCloud(new pcl::PointCloud<pcl::PointXYZI>);
pcl::ModelCoefficients::Ptr plane_coeff(new pcl::ModelCoefficients);

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
    seg.setDistanceThreshold(0.05);

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

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud)
{
  // ----------------------------------------------------
  // -----Open 3D viewer and display City Block     -----
  // ----------------------------------------------------

  //ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
  //pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
//   inputCloud = pointProcessorI->FilterCloud(inputCloud, 0.3, Eigen::Vector4f (-40, -10, -2, 1), Eigen::Vector4f ( 50, 20, 2, 1));
    inputCloud = pointProcessorI->FilterCloud(inputCloud, 0.17, Eigen::Vector4f (0, -40, -2, 1), Eigen::Vector4f (  200, 40, 10, 1));

    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr,pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentedPlane = pointProcessorI->RANSAC_PlaneSegment_Scratch(inputCloud,50,0.25);

    // ----------------------
    plane_coeff = mGet_plane_coeff( segmentedPlane.second );
    std::pair<Eigen::Matrix3d, Eigen::Vector3d> plane_RT = mGet_plane_Rot_Tran(plane_coeff);
    // ----------------------
    std::pair<Eigen::Quaterniond, Eigen::Vector3d> plane_Quaternion = mGet_plane_qauternion(plane_coeff);
    Eigen::Quaternionf plane_f_quaternion = plane_Quaternion.first.cast<float>();
    Eigen::Vector3f plane_t_vector = -plane_Quaternion.second.cast<float>();
    // std::cout << "quaternion is: " << plane_f_quaternion.coeffs() << std::endl;
    // ----------------------
        
    renderPointCloud(viewer, segmentedPlane.first,"Obstacle Cloud",Color(0,0,1));
    renderPointCloud(viewer, segmentedPlane.second,"Plane Cloud",Color(1,0,1));
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->EuclideanCluster_scratch_clustering(segmentedPlane.first, 0.5, 11, 900);
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};

    // // Define the minimum and maximum corners of the cube
    // double width = 4.3;       // depth
    // double height = 2.0;      // width
    // double depth = 1.4;      // heigh

    // viewer->addCube(plane_t_vector, plane_f_quaternion, width, height, depth, "bbox");
    // viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 1.0, "bbox");
    // viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.3, "bbox");
    
    std::chrono::milliseconds melapsedTime(0);
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

        pcl::PointCloud<pcl::PointXYZ>::Ptr bb_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        Eigen::Vector3f bp1 = Eigen::Vector3f(box.x_min, box.y_min, box.z_min);
        Eigen::Vector3f bp7 = Eigen::Vector3f(box.x_max, box.y_max, box.z_max);

        bb_cloud->push_back( pcl::PointXYZ( bp1(0), bp1(1), bp1(2))); //bp1 - 0
        bb_cloud->push_back( pcl::PointXYZ( bp7(0), bp1(1), bp1(2))); //bp2 - 1 
        bb_cloud->push_back( pcl::PointXYZ( bp7(0), bp7(1), bp1(2))); //bp3 - 2
        bb_cloud->push_back( pcl::PointXYZ( bp1(0), bp7(1), bp1(2))); //bp4 - 3
        bb_cloud->push_back( pcl::PointXYZ( bp1(0), bp1(1), bp7(2))); //bp5 - 4
        bb_cloud->push_back( pcl::PointXYZ( bp7(0), bp1(1), bp7(2))); //bp6 - 5
        bb_cloud->push_back( pcl::PointXYZ( bp7(0), bp7(1), bp7(2))); //bp7 - 6
        bb_cloud->push_back( pcl::PointXYZ( bp1(0), bp7(1), bp7(2))); //bp8 - 7

        tranformation_mat.block<3,3>(0,0) = plane_RT.first.inverse();
        tranformation_mat.block<3,1>(0,3) = -plane_RT.second;

        pcl::PointCloud<pcl::PointXYZ>::Ptr bb_cloud_transformed(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*bb_cloud, *bb_cloud_transformed, tranformation_mat);        

        auto mendTime = std::chrono::steady_clock::now();
        melapsedTime = melapsedTime + std::chrono::duration_cast<std::chrono::milliseconds>(mendTime - mstartTime);
        // // ------------------------
        // pcl::PointCloud<pcl::PointXYZ>::Ptr bb_cloud_transformed(new pcl::PointCloud<pcl::PointXYZ>);
        // bb_cloud_transformed = mGet_boundingBox_8pts(cluster);

        Eigen::Vector3d bp_base( bb_cloud_transformed->points[0].x, bb_cloud_transformed->points[0].y,  bb_cloud_transformed->points[0].z );
        Eigen::Vector3d bp_top( bb_cloud_transformed->points[4].x, bb_cloud_transformed->points[4].y,  bb_cloud_transformed->points[4].z );
        Eigen::Vector3d bp_width( bb_cloud_transformed->points[3].x, bb_cloud_transformed->points[3].y,  bb_cloud_transformed->points[3].z );
        Eigen::Vector3d bp_lenght( bb_cloud_transformed->points[1].x, bb_cloud_transformed->points[1].y,  bb_cloud_transformed->points[1].z );
        float obj_height = (bp_top - bp_base).norm(), obj_width = (bp_width - bp_base).norm(), obj_lenght = (bp_lenght - bp_base).norm();
        
        // if( (1.1 < obj_height && obj_height < 2.5) && (1.1 < obj_width && obj_width < 5.0) && ( obj_lenght < 5.0)  ) // check object height
        // {
        //     Color mcol( 0.5, 1.0, 0.0);
        //     renderBox_by_pointcloud_line(viewer, bb_cloud_transformed, clusterId, mcol, 0.1);
        // }
        // else
        //     if( 0.5 < obj_height && 0.1 < (obj_height/obj_lenght) ) // check object height
        //     {
        //         Color mcol( 0.5, 0.5, 0.5);
        //         renderBox_by_pointcloud_line(viewer, bb_cloud_transformed, clusterId, mcol, 0.1);
        //     }
        //     else
        //     {
        //         Color mcol( 0.0, 0.4, 0.4);
        //         renderBox_by_pointcloud_line(viewer, bb_cloud_transformed, clusterId, mcol, 0.1);
            // }
        // renderBox_by_pointcloud(viewer, bb_cld_t, clusterId, mcol, 0.1);
        // ----------------------
        // Box box=pointProcessorI->BoundingBox(cluster);
        // Box box=pointProcessorI->BoundingBox(transformed_cluster);
        // renderBox(viewer,box,clusterId);
        ++clusterId;
    }
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
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr,pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentedPlane = pointProcess->SegmentPlane(inputCloud,100,1);

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

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    // CameraAngle setAngle = XY;
    // CameraAngle setAngle = BehindCar;
    CameraAngle setAngle = FromCamParamFile;

    initCamera(setAngle, viewer);
    //simpleHighway(viewer);
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    // std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_2");
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("/home/reamond/Documents/pcd_data");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;
    
    while (!viewer->wasStopped ())
    {
          // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // -----------------
        // viewer->addPlane(*plane_coeff, "plane");
        // -----------------

        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessorI, inputCloudI);

        streamIterator++;
        if(streamIterator == stream.end())
            streamIterator = stream.begin();

        viewer->spinOnce();
        // viewer->saveCameraParameters("cameraParameter.txt");
    } 
}
