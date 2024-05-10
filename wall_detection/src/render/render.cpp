/* \author Aaron Brown */
// Functions and structs used to render the enviroment
// such as cars and the highway

#include "render.h"

void renderHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{

	// units in meters
	double roadLength = 50.0;
	double roadWidth = 12.0;
	double roadHeight = 0.2;

	viewer->addCube(-roadLength/2, roadLength/2, -roadWidth/2, roadWidth/2, -roadHeight, 0, .2, .2, .2, "highwayPavement"); 
  	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, "highwayPavement"); 
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, .2, .2, .2, "highwayPavement");
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1.0, "highwayPavement");
	viewer->addLine(pcl::PointXYZ(-roadLength/2,-roadWidth/6, 0.01),pcl::PointXYZ(roadLength/2, -roadWidth/6, 0.01),1,1,0,"line1");
	viewer->addLine(pcl::PointXYZ(-roadLength/2, roadWidth/6, 0.01),pcl::PointXYZ(roadLength/2, roadWidth/6, 0.01),1,1,0,"line2");
}

int countRays = 0;
void renderRays(pcl::visualization::PCLVisualizer::Ptr& viewer, const Vect3& origin, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{

	for(pcl::PointXYZ point : cloud->points)
	{
		viewer->addLine(pcl::PointXYZ(origin.x, origin.y, origin.z), point,1,0,0,"ray"+std::to_string(countRays));
		countRays++;
	}
}

void clearRays(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
	while(countRays)
	{
		countRays--;
		viewer->removeShape("ray"+std::to_string(countRays));
	}
}

void renderPointCloud(pcl::visualization::PCLVisualizer::Ptr& viewer, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, std::string name, Color color)
{

  	viewer->addPointCloud<pcl::PointXYZ> (cloud, name);
  	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, name);
  	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, name);
}

void renderPointCloud(pcl::visualization::PCLVisualizer::Ptr& viewer, const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, std::string name, Color color)
{

	if(color.r==-1)
	{
		// Select color based off of cloud intensity
		pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity_distribution(cloud,"intensity");
  		viewer->addPointCloud<pcl::PointXYZI>(cloud, intensity_distribution, name);
	}
	else
	{
		// Select color based off input value
		viewer->addPointCloud<pcl::PointXYZI> (cloud, name);
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, name);
	}

	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, name);
}

// Draw wire frame box with filled transparent color 
void renderBox(pcl::visualization::PCLVisualizer::Ptr& viewer, Box box, int id, Color color, float opacity)
{
	if(opacity > 1.0)
		opacity = 1.0;
	if(opacity < 0.0)
		opacity = 0.0;
	
	std::string cube = "box"+std::to_string(id);
    //viewer->addCube(box.bboxTransform, box.bboxQuaternion, box.cube_length, box.cube_width, box.cube_height, cube);
    viewer->addCube(box.x_min, box.x_max, box.y_min, box.y_max, box.z_min, box.z_max, color.r, color.g, color.b, cube);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, cube); 
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, cube);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity, cube);
    
    std::string cubeFill = "boxFill"+std::to_string(id);
    //viewer->addCube(box.bboxTransform, box.bboxQuaternion, box.cube_length, box.cube_width, box.cube_height, cubeFill);
    viewer->addCube(box.x_min, box.x_max, box.y_min, box.y_max, box.z_min, box.z_max, color.r, color.g, color.b, cubeFill);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, cubeFill); 
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, cubeFill);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity*0.3, cubeFill);
}

void renderBox(pcl::visualization::PCLVisualizer::Ptr& viewer, BoxQ box, int id, Color color, float opacity)
{
	if(opacity > 1.0)
		opacity = 1.0;
	if(opacity < 0.0)
		opacity = 0.0;
	
	std::string cube = "box"+std::to_string(id);
    viewer->addCube(box.bboxTransform, box.bboxQuaternion, box.cube_length, box.cube_width, box.cube_height, cube);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, cube); 
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, cube);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity, cube);
    
    std::string cubeFill = "boxFill"+std::to_string(id);
    viewer->addCube(box.bboxTransform, box.bboxQuaternion, box.cube_length, box.cube_width, box.cube_height, cubeFill);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, cubeFill); 
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, cubeFill);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity*0.3, cubeFill);
}

void renderBox_params(pcl::visualization::PCLVisualizer::Ptr& viewer, Box_params box, int id, Color color, float opacity)
{
	std::string cube = "box"+std::to_string(id);
	viewer->addCube(box.center, box.quaternion, box.lenght, box.width, box.height, cube);
	// viewer->addCube(box.center, box.quaternion, box.height, box.width, box.lenght, cube);
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, cube);
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity, cube);
}

void renderBox_by_pointcloud(pcl::visualization::PCLVisualizer::Ptr& viewer, pcl::PointCloud<pcl::PointXYZ>::Ptr &corners, int id, Color color, float opacity)
{
	if(opacity > 1.0)
		opacity = 1.0;
	if(opacity < 0.0)
		opacity = 0.0;
	
	std::string cube = "box"+std::to_string(id);
	    
    // Define the faces of the cube
    std::vector<pcl::Vertices> faces(6);
    faces[0].vertices = {0, 1, 5, 4};   // Face 1 (Front)
    faces[1].vertices = {4, 5, 6, 7};   // Face 2 (Back)
    faces[2].vertices = {7, 6, 2, 3};   // Face 3 (Bottom)
    faces[3].vertices = {3, 2, 1, 0};   // Face 4 (Right)
    faces[4].vertices = {3, 0, 4, 7};   // Face 5 (Top)
    faces[5].vertices = {1, 2, 6, 5};   // Face 6 (Left)

	// Create the polygon mesh
    pcl::PolygonMesh mesh;
    mesh.polygons = faces;

	pcl::PCLPointCloud2::Ptr plane_cloud_blob(new pcl::PCLPointCloud2);
	pcl::toPCLPointCloud2(*corners, *plane_cloud_blob);
	mesh.cloud = *plane_cloud_blob;

    // Add the mesh to the viewer
    viewer->addPolygonMesh(mesh, cube);
    // viewer->addCube(box.bboxTransform, box.bboxQuaternion, box.cube_length, box.cube_width, box.cube_height, cube);
    // viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, cube); 
    // viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, cube);
    // viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity, cube);
    
    // std::string cubeFill = "boxFill"+std::to_string(id);
    // // viewer->addCube(box.bboxTransform, box.bboxQuaternion, box.cube_length, box.cube_width, box.cube_height, cubeFill);
    // viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, cubeFill); 
    // viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, cubeFill);
    // viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity*0.3, cubeFill);
}

void renderBox_by_pointcloud_line(pcl::visualization::PCLVisualizer::Ptr& viewer, pcl::PointCloud<pcl::PointXYZ>::Ptr &corners, int id, Color color, float opacity)
{
	if(opacity > 1.0)
		opacity = 1.0;
	if(opacity < 0.0)
		opacity = 0.0;
	
	std::string cube = "box"+std::to_string(id);
	    
    // Add the cube corners to the viewer
    viewer->addLine(corners->points[0], corners->points[1], color.r, color.g, color.b, cube+"01");
    viewer->addLine(corners->points[1], corners->points[2], color.r, color.g, color.b, cube+"02");
	viewer->addLine(corners->points[2], corners->points[3], color.r, color.g, color.b, cube+"03");
	viewer->addLine(corners->points[3], corners->points[0], color.r, color.g, color.b, cube+"04");

	viewer->addLine(corners->points[4], corners->points[5], color.r, color.g, color.b, cube+"05");
    viewer->addLine(corners->points[5], corners->points[6], color.r, color.g, color.b, cube+"06");
	viewer->addLine(corners->points[6], corners->points[7], color.r, color.g, color.b, cube+"07");
	viewer->addLine(corners->points[7], corners->points[4], color.r, color.g, color.b, cube+"08");

	viewer->addLine(corners->points[0], corners->points[4], color.r, color.g, color.b, cube+"09");
    viewer->addLine(corners->points[1], corners->points[5], color.r, color.g, color.b, cube+"10");
	viewer->addLine(corners->points[2], corners->points[6], color.r, color.g, color.b, cube+"11");
	viewer->addLine(corners->points[3], corners->points[7], color.r, color.g, color.b, cube+"12");
}
