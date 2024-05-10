#ifndef BOX_H
#define BOX_H
#include <Eigen/Geometry> 

struct BoxQ
{
	Eigen::Vector3f bboxTransform;
	Eigen::Quaternionf bboxQuaternion;
	float cube_length;
    float cube_width;
    float cube_height;
};
struct Box
{
	float x_min;
	float y_min;
	float z_min;
	float x_max;
	float y_max;
	float z_max;
};

struct Box_params
{
	Eigen::Quaternionf quaternion;
	Eigen::Vector3f min;
	Eigen::Vector3f max;
	Eigen::Vector3f center;
	double dist_to_road;
	double width;
	double height;
	double lenght;
};



// /*
// 	  bp7   *----------*  bp6
// 		   /|	   	  /|
// 	bp8   *----------* | bp5
//           | |        | |
// 	  bp3 | *----------*  bp2
// 	      |/		  |/
// 	bp4   *----------* bp1
// */
// struct Box_8pts
// {
// 	Eigen::Vector3d bp1;
// 	Eigen::Vector3d bp2;
// 	Eigen::Vector3d bp3;
// 	Eigen::Vector3d bp4;
// 	Eigen::Vector3d bp5;
// 	Eigen::Vector3d bp6;
// 	Eigen::Vector3d bp7;
// 	Eigen::Vector3d bp8;
// };
#endif