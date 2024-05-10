// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include <unordered_set>
#include <Eigen/Dense>

using namespace Eigen;
//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}

template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << "Total Number of Points "<<cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    pcl::VoxelGrid<PointT> voxgrid;
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);

    voxgrid.setInputCloud (cloud);
    voxgrid.setLeafSize (filterRes, filterRes, filterRes);
    voxgrid.filter (*cloud_filtered);


    typename pcl::PointCloud<PointT>::Ptr cloud_region (new pcl::PointCloud<PointT>);
    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloud_filtered);
    region.filter(*cloud_region);

    std::vector<int> indx;

    pcl::CropBox<PointT> roof (true);
    roof.setMin(Eigen::Vector4f (-1.5,-1.7,-1,1));
    roof.setMax(Eigen::Vector4f (2.6,1.7,-4,1));
    roof.setInputCloud(cloud_region);
    roof.filter(indx);

    pcl::PointIndices::Ptr in (new pcl::PointIndices);
    for (int p:indx){
        in->indices.push_back(p);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud_region);
    extract.setIndices(in);
    extract.setNegative(true);
    extract.filter(*cloud_region);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_region;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
  typename pcl::PointCloud<PointT>::Ptr ObstCloud (new pcl::PointCloud<PointT>());
  typename pcl::PointCloud<PointT>::Ptr PlaneCloud (new pcl::PointCloud<PointT>());
  
  for(int index:inliers->indices){
      PlaneCloud->points.push_back(cloud->points[index]);
  }
  
  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud (cloud);
  extract.setIndices (inliers);
  extract.setNegative (true);
  //filter out points that are not in plane.
  extract.filter (*ObstCloud);

  std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(ObstCloud, PlaneCloud);
  return segResult;
}


/////////////////////// SEGMENT PLANE RANSACE FROM SCRATCH/////////////////////////////////////////////////////////////

template<typename PointT>
std::pair< pcl::ModelCoefficients::Ptr,  std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>> ProcessPointClouds<PointT>::RANSAC_PlaneSegment_Scratch(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float disttanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    pcl::ModelCoefficients::Ptr plane_coefficient (new pcl::ModelCoefficients);
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	PointT p1;
	PointT p2;
	PointT p3;
	int idx1;
	int idx2;
	int idx3;
	float a,b,c,d,dist,denominator;

	// For max iterations
	for(int it=0;it<maxIterations;it++)
	{
		std::unordered_set<int> inliers;
		/*Identify 3 points randomly*/
		while(inliers.size()<3)
			inliers.insert((rand() % cloud->points.size()));
		auto iter = inliers.begin();
		idx1 = *iter;
		++iter;
		idx2 = *iter;
		++iter;
		idx3 = *iter;

		p1 = cloud->points[idx1];
		p2 = cloud->points[idx2];
		p3 = cloud->points[idx3];

		/*Fit a plane using the above 3 points*/
		a = (((p2.y-p1.y)*(p3.z-p1.z))-((p2.z-p1.z)*(p3.y-p1.y)));
		b = (((p2.z-p1.z)*(p3.x-p1.x))-((p2.x-p1.x)*(p3.z-p1.z)));
		c = (((p2.x-p1.x)*(p3.y-p1.y))-((p2.y-p1.y)*(p3.x-p1.x)));
		d = -(a*p1.x+b*p1.y+c*p1.z);
		denominator = sqrt(a*a+b*b+c*c);

		// Measure disttance between every point and fitted plane
		for(int pt_cnt=0;pt_cnt<cloud->points.size();pt_cnt++)
		{
			if(pt_cnt!=idx1||pt_cnt!=idx2||pt_cnt!=idx3)
			{
				dist = (fabs(a*cloud->points[pt_cnt].x+b*cloud->points[pt_cnt].y+c*cloud->points[pt_cnt].z+d)/denominator);
				// If disttance is smaller than threshold count it as inlier
				if(dist<=disttanceThreshold)
				{
					inliers.insert(pt_cnt);
				}
			}
		}

		/*Store the temporary buffer if the size if more than previously idenfitied points */
		if(inliers.size()>inliersResult.size())
		{
			inliersResult.clear();
			inliersResult = inliers;
            plane_coefficient->values.clear();
            plane_coefficient->values.push_back(a);
            plane_coefficient->values.push_back(b);
            plane_coefficient->values.push_back(c);
            plane_coefficient->values.push_back(d);
		}

	}

	// Segment the largest planar component from the remaining cloud
	if (inliersResult.size () == 0)
	{
	  std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
	}
    // else
    //     std::cout << "plane coeffs are: a=" << a << ", b=" << b << ", c=" << c << ", d=" << d << std::endl;
	typename pcl::PointCloud<PointT>::Ptr cloudInliers(new pcl::PointCloud<PointT>());
	typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new pcl::PointCloud<PointT>());

	/*Copy the points from inputcloud in to cloudInliers if the indices is in inliersResult vector
	 * or else copy the point to cloudOutliers*/
	for(int index = 0; index < cloud->points.size(); index++)
	{
		PointT point = cloud->points[index];
		if(inliersResult.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}
	/*Create a pair using inlier and outlier points*/
	std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloudOutliers, cloudInliers);
    std::pair<typename pcl::ModelCoefficients::Ptr,  std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>> allResult(plane_coefficient, segResult);
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    return allResult;
}



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////// SEGMENT PLANE with RANSACE using initial value FROM SCRATCH /////////////////////////////////////////////////////////////

template<typename PointT>
std::pair< pcl::ModelCoefficients::Ptr,  std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>> ProcessPointClouds<PointT>::RANSAC_PlaneSegment_with_init_Scratch(typename pcl::PointCloud<PointT>::Ptr cloud, pcl::ModelCoefficients::Ptr plane_coeff, int maxIterations, float disttanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	PointT p1;
	PointT p2;
	PointT p3;
	int idx1;
	int idx2;
	int idx3;
	float a,b,c,d,dist,denominator;
    // ++++++++++++++++++++++++++++++++++++++++++
    Eigen::Vector3d init_plane_nvec(plane_coeff->values[0], plane_coeff->values[1], plane_coeff->values[2]);
    // ++++++++++++++++++++++++++++++++++++++++++
	
    // For max iterations
	for(int it=0;it<maxIterations;it++)
	{
        std::unordered_set<int> inliers;
        if (it == 0)
        {
            a = plane_coeff->values[0];
            b = plane_coeff->values[1];
            c = plane_coeff->values[2];
            d = plane_coeff->values[3];
        }
        else
        {
            float ang_diff = 0;
            do
            {
                inliers.clear();
                /*Identify 3 points randomly*/
                while (inliers.size() < 3)
                    inliers.insert((rand() % cloud->points.size()));
                auto iter = inliers.begin();
                idx1 = *iter;
                ++iter;
                idx2 = *iter;
                ++iter;
                idx3 = *iter;

                p1 = cloud->points[idx1];
                p2 = cloud->points[idx2];
                p3 = cloud->points[idx3];

                /*Fit a plane using the above 3 points*/
                a = (((p2.y - p1.y) * (p3.z - p1.z)) - ((p2.z - p1.z) * (p3.y - p1.y)));
                b = (((p2.z - p1.z) * (p3.x - p1.x)) - ((p2.x - p1.x) * (p3.z - p1.z)));
                c = (((p2.x - p1.x) * (p3.y - p1.y)) - ((p2.y - p1.y) * (p3.x - p1.x)));
                d = -(a * p1.x + b * p1.y + c * p1.z);
                // ++++++++++++++++++++++++++++++++++++++++++
                Eigen::Vector3d fit_plane_nvec(a, b, c);
                float dot_v1_v2 = init_plane_nvec.dot(fit_plane_nvec);
                ang_diff = acos(dot_v1_v2 / (init_plane_nvec.norm() * fit_plane_nvec.norm()));
                // ++++++++++++++++++++++++++++++++++++++++++
            } while (ang_diff > 0.5);
        }
        denominator = sqrt(a*a+b*b+c*c);
		// Measure disttance between every point and fitted plane
		for(int pt_cnt=0;pt_cnt<cloud->points.size();pt_cnt++)
		{
			if(pt_cnt!=idx1||pt_cnt!=idx2||pt_cnt!=idx3)
			{
				dist = (fabs(a*cloud->points[pt_cnt].x+b*cloud->points[pt_cnt].y+c*cloud->points[pt_cnt].z+d)/denominator);
				// If disttance is smaller than threshold count it as inlier
				if(dist<=disttanceThreshold)
				{
					inliers.insert(pt_cnt);
				}
			}
		}

		/*Store the temporary buffer if the size if more than previously idenfitied points */
		if(inliers.size()>inliersResult.size())
		{
			inliersResult.clear();
			inliersResult = inliers;
            plane_coeff->values.clear();
            plane_coeff->values.push_back(a);
            plane_coeff->values.push_back(b);
            plane_coeff->values.push_back(c);
            plane_coeff->values.push_back(d);

		}

	}

	// Segment the largest planar component from the remaining cloud
	if (inliersResult.size () == 0)
	{
	  std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
	}
    // else
    //     std::cout << "plane coeffs are: a=" << a << ", b=" << b << ", c=" << c << ", d=" << d << std::endl;
	typename pcl::PointCloud<PointT>::Ptr cloudInliers(new pcl::PointCloud<PointT>());
	typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new pcl::PointCloud<PointT>());

	/*Copy the points from inputcloud in to cloudInliers if the indices is in inliersResult vector
	 * or else copy the point to cloudOutliers*/
	for(int index = 0; index < cloud->points.size(); index++)
	{
		PointT point = cloud->points[index];
		if(inliersResult.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}
	/*Create a pair using inlier and outlier points*/
	std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloudOutliers, cloudInliers);

    std::pair<typename pcl::ModelCoefficients::Ptr,  std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>> allResult(plane_coeff, segResult);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    return allResult;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float disttanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices());
    // TODO:: Fill in this function to find inliers for the cloud.
    
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (maxIterations);
    seg.setDistanceThreshold (disttanceThreshold);

    //Segmenting Largest Planar component from input cloud
    seg.setInputCloud(cloud);
    //segment the cloud based on inliers
    seg.segment(*inliers,*coefficients);
    
    
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;
    if(inliers->indices.size()==0){
        std::cout<<"Could not generate planar model for the given data set!"<<std::endl;
    }
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles

    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (clusterTolerance); // 2cm
    ec.setMinClusterSize (minSize);
    ec.setMaxClusterSize (maxSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);

    for (pcl::PointIndices getIndices : cluster_indices){
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);

        for(int index : getIndices.indices)
            cloud_cluster->points.push_back(cloud->points[index]);
        
        cloud_cluster->width=cloud_cluster->points.size();
        cloud_cluster->height=1;
        cloud_cluster->is_dense=true;

        clusters.push_back(cloud_cluster);
        
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

////////////////////////////////////////////////////// EUCLIDEAN CLUSTERING FROM SCRATCH ////////////////////////////////////////////////////////////////////////////



/* a recurssive function.
   Proximity_Point function look points distanceTol
 * distance from the given point in the cloud and return the indices of the points
 *  
 * 1. If the target point is not processed then set is as processed and search
 * 2. the KDTree for all the points within the distanceTol
 * 3. Use each of the nearby points and search for other points that are within
 * 4. distanceTol distance from this points
 *
 * */
template<typename PointT>
void ProcessPointClouds<PointT>::Proximity_Points(typename pcl::PointCloud<PointT>::Ptr cloud,std::vector<int> &cluster,std::vector<bool> &processed_pts,int idx,typename KdTree_Scratch<PointT>::KdTree_Scratch* tree,float distanceTol, int maxSize)
{
	if((processed_pts[idx]==false)&&
			(cluster.size()<maxSize))
	{
		processed_pts[idx]=true;
		cluster.push_back(idx);
		std::vector<int> nearby = tree->search(cloud->points[idx],distanceTol);
		for(int index : nearby)
		{
			if(processed_pts[index]==false)
			{
				Proximity_Points(cloud, cluster,processed_pts,index,tree,distanceTol,maxSize);
			}
		}
	}

}


/* euclideanCluster function looks for clusters that have points within min and max limits
 * 1. Take one point at a time from the cluster , call Proximity function to identify the
 * 2. list of points that are within distanceTol limits
 * 3. Check if the no of points in cluster ,returned by proximity function, are in (minSize, maxSize)
 * 4. limits if not discard
 * */


template<typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::EuclideanCluster_Scratch(typename pcl::PointCloud<PointT>::Ptr cloud, typename KdTree_Scratch<PointT>::KdTree_Scratch* tree, float distanceTol, int minSize, int maxSize)
{
	std::vector<std::vector<int>> clusters;
	/*Create a flag for each point in the cloud, to identified if the point is processed or not, and set it to false*/
	std::vector<bool> processed_ptslag(cloud->points.size(),false);

	/*Loop through each point of the cloud*/
	for(int idx=0;idx<cloud->points.size();idx++)
	{
		/*Pass the point to Proximity function only if it was not processed
		 * (either added to a cluster or discarded)*/
		if(processed_ptslag[idx]==false)
		{
			std::vector<int> cluster;
			Proximity_Points(cloud, cluster,processed_ptslag,idx,tree,distanceTol,maxSize);

			/*Check if the number of points in the identified cluster are with in limits */
			if((cluster.size()>=minSize)&&cluster.size()<=maxSize)
				clusters.push_back(cluster);
		}

	}
	return clusters;

}


/* Clustering_euclideanCluster function shall identify the cluster of point that have given no of min, max points and meet the cluster tolerance requirement.
 * 1. Using points in the given cloud KDTree is formed.
 * 2. Using Euclidena Clustering, clusters are searched in the created KDTree
 * 3. Identified clusters are filtered, clusters that dont have points in min, max points are discarded.
 * */

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::EuclideanCluster_scratch_clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // Create the KdTree object using the points in cloud.
    typename KdTree_Scratch<PointT>::KdTree_Scratch *tree =new KdTree_Scratch<PointT>;
    tree->insert(cloud);

    //perform euclidean clustering to group detected obstacles
	std::vector<std::vector<int>> cluster_indices = EuclideanCluster_Scratch(cloud, tree,clusterTolerance ,minSize,maxSize);

	for (std::vector<std::vector<int>>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
	  {
		typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
	    for (std::vector<int>::const_iterator pit = it->begin (); pit != it->end (); ++pit)
	      cloud_cluster->points.push_back (cloud->points[*pit]); //*
	    cloud_cluster->width = cloud_cluster->points.size ();
	    cloud_cluster->height = 1;
	    cloud_cluster->is_dense = true;

	    clusters.push_back(cloud_cluster);
	  }
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "euclideanClustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}

template<typename PointT>
Box_params ProcessPointClouds<PointT>::BoundingBox_params(typename pcl::PointCloud<PointT>::Ptr cluster)
{
    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);
    Box_params box;
    box.min << minPoint.x, minPoint.y, minPoint.z;
    box.max << maxPoint.x, maxPoint.y, maxPoint.z;
    // box.center = (box.max+box.min)/2;
    // +++ --- +++ --- --- --- --- --- --- --- +++ --- +++
    // box.width = abs((box.max-box.min)(0));
    // box.lenght = abs((box.max-box.min)(1));
    // box.height = abs((box.max-box.min)(2));
    // +++ --- +++ --- --- --- --- --- --- --- +++ --- +++
    box.lenght = abs((box.max-box.min)(0));
    box.width = abs((box.max-box.min)(1));
    box.height = abs((box.max-box.min)(2));
    box.dist_to_road = minPoint.z;

    return box;
}

template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}

template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{
    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}

template<typename PointT>
std::pair<line_data, std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>> ProcessPointClouds<PointT>::RANSAC_LineSegment_Scratch(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    line_data line_coeff;
    // Create a set to store inliers of the best line model
    std::unordered_set<int> inliersResult;

    // Random seed initialization
    srand(time(NULL));
 
    // Variables to store two random points and their indices
    PointT p1, p2;
    int idx1, idx2;
    // std::cout << " r mur" << std::endl;
    // Variables for line coefficients
    float a, b, c, dist, denominator;
    //Eigen::VectorXf coff;
    // Perform RANSAC iterations
    for(int it = 0; it < maxIterations; it++)
    {
        std::unordered_set<int> inliers;
        // std::cout << " 10r mur" << std::endl;
        // Randomly select two unique points
        while(inliers.size() < 2)
            inliers.insert((rand() % cloud->points.size()));
        auto iter = inliers.begin();
        idx1 = *iter;
        ++iter;
        idx2 = *iter;
        // std::cout << " random point owned" << std::endl;
        // Extract selected points
        p1 = cloud->points[idx1];
      //  std::cout << " random point owned1" << std::endl;
        p2 = cloud->points[idx2];
       // std::cout << " random point owned2" << std::endl;
        // Compute line coefficients using selected points
        a = p2.y - p1.y;
      //  std::cout << " random point owned3" << std::endl;
        b = p1.x - p2.x;
        c = p2.x * p1.y - p1.x * p2.y;
        // std::cout << " denominator prev" << std::endl;
        denominator = sqrt(a * a + b * b);
        // coefficients[0]=a;
        // coefficients[1]=b;
        // coefficients[2]=c;
        // std::cout << " coff calculated" << std::endl;
        // Measure distance between every point and fitted line
        for(int pt_cnt = 0; pt_cnt < cloud->points.size(); pt_cnt++)
        {   
           // std::cout << pt_cnt << std::endl;
            if(pt_cnt != idx1 && pt_cnt != idx2)
            {
               // std::cout << " if true" << std::endl;
                dist = fabs(a * cloud->points[pt_cnt].x + b * cloud->points[pt_cnt].y + c) / denominator;
                // If distance is smaller than threshold, count it as inlier
                if(dist <= distanceThreshold)
                {
                    inliers.insert(pt_cnt);
                }
            }
        }

        // Store the inliers of the best model so far
        if(inliers.size() > inliersResult.size())
        {
            inliersResult.clear();
            inliersResult = inliers;
            line_coeff.a=a;
            line_coeff.b=b;
            line_coeff.c=c;
            line_coeff.p1=p1;
            line_coeff.p2=p2;
            line_coeff.line_dist=fabs(c)/denominator;
            line_coeff.angle_x=atan2(line_coeff.a, line_coeff.b)/3.14*180;
            if(line_coeff.angle_x<0){
                line_coeff.angle_x+=180;
            }
            if(abs(line_coeff.angle_x)>75 && abs(line_coeff.angle_x)<105){
                //right or left
                line_coeff.x_inter=-line_coeff.c/line_coeff.a;
                if(line_coeff.x_inter<0){
                    line_coeff.line_stat=2; //left line
                }
                else{
                    line_coeff.line_stat=4; //right line
                }
            }

            else if (abs(line_coeff.angle_x)>165 || abs( line_coeff.angle_x)<15){
                //front or back
                line_coeff.y_inter=-line_coeff.c/line_coeff.b;
                if(line_coeff.y_inter>0){
                    line_coeff.line_stat=1; //front line
                }
                else{
                    line_coeff.line_stat=3; //back line
                }
            }
            else{
                line_coeff.line_stat=0;
            }
            line_coeff.m = -line_coeff.a/line_coeff.b;
            line_coeff.r = -line_coeff.c/line_coeff.b;
            line_coeff.k = -line_coeff.b/line_coeff.a;
            line_coeff.l = -line_coeff.c/line_coeff.a;
            }
    }
    // std::cout << "line segmented "  << std::endl;
    // Create point cloud pointers for inliers and outliers
    typename pcl::PointCloud<PointT>::Ptr cloudInliers(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new pcl::PointCloud<PointT>()); 

    // Copy points to inliers or outliers based on inliers set
    for(int index = 0; index < cloud->points.size(); index++)
    {
        PointT point = cloud->points[index];
        if(inliersResult.count(index))
            cloudInliers->points.push_back(point);
        else
            cloudOutliers->points.push_back(point);
    }

    // Create a pair using inlier and outlier point clouds
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloudOutliers, cloudInliers);
    std::pair<line_data, std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>> allResult(line_coeff,segResult);
    // Measure time taken for line segmentation
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "Line segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;
    //coefficients=coff;
    return allResult;
}

// double determinant(double mat[3][3]){
//     double ans;
//     ans = mat[0][0] * (mat[1][1] * mat[2][2] - mat[2][1] * mat[1][2])
//           - mat[0][1] * (mat[1][0] * mat[2][2] - mat[1][2] * mat[2][0])
//           + mat[0][2] * (mat[1][0] * mat[2][1] - mat[1][1] * mat[2][0]);
//     return ans;
// }


template<typename PointT>
PointT findSolution(Eigen::Matrix4d coeff) {
    // Matrix d using coeff as given in Cramer's rule
    Eigen::Matrix3d d;
    d <<  coeff(0,0), coeff(0,1), coeff(0,2),
          coeff(1,0), coeff(1,1), coeff(1,2),
          coeff(2,0), coeff(2,1), coeff(2,2);
    
    // Matrix d1 using coeff as given in Cramer's rule
    Eigen::Matrix3d d1;
    d1 <<   coeff(0,3), coeff(0,1), coeff(0,2),
            coeff(1,3), coeff(1,1), coeff(1,2),
            coeff(2,3), coeff(2,1), coeff(2,2);
            
    // Matrix d2 using coeff as given in Cramer's rule
    Eigen::Matrix3d d2;
    d2 <<   coeff(0,0), coeff(0,3), coeff(0,2),
            coeff(1,0), coeff(1,3), coeff(1,2),
            coeff(2,0), coeff(2,3), coeff(2,2);
 
    // Matrix d3 using coeff as given in Cramer's rule
    Eigen::Matrix3d d3;
    d3 << coeff(0,0), coeff(0,1), coeff(0,3),
          coeff(1,0), coeff(1,1), coeff(1,3),
          coeff(2,0), coeff(2,1), coeff(2,3);
 
    // Calculating Determinant of Matrices d, d1, d2, d3
    double D = d.determinant();
    double D1 = d1.determinant();
    double D2 = d2.determinant();
    double D3 = d3.determinant();
 
    // Case 1
    if (D != 0) {
        // Coeff have a unique solution. Apply Cramer's Rule
        PointT p0;
        p0.x = D1 / D;
        p0.y = D2 / D;
        p0.z = D3 / D; // calculating z using Cramer's rule
        return p0;
    }
    // Case 2
    else {
        PointT p0;
        p0.x = 0;
        p0.y = 0;
        p0.z = 0; // calculating z using Cramer's rule
        return p0;
    }
}


template<typename PointT>
std::pair< PointT, std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>> ProcessPointClouds<PointT>::RANSAC_SphereSegment_Scratch(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float disttanceThreshold, float sphereRadius)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	PointT p1;
	PointT p2;
	PointT p3;
    PointT pT;
	int idx1;
	int idx2;
	int idx3;
	float a,b,c;
    Eigen::Matrix4d coeff;
	// For max iterations
	for(int it=0;it<maxIterations;it++)
	{
		std::unordered_set<int> inliers;
        
		while(inliers.size()<3)
			inliers.insert((rand() % cloud->points.size()));
		auto iter = inliers.begin();
		idx1 = *iter;
		++iter;
		idx2 = *iter;
		++iter;
		idx3 = *iter;

		p1 = cloud->points[idx1];
		p2 = cloud->points[idx2];
		p3 = cloud->points[idx3];
        // std::cout << "----------------------------------------------------------" << std::endl;
        // std::cout << p1 << ":" << p2 << ":" << p3 << ":" << std::endl; 
		/*Fit a plane using the above 3 points*/
		coeff(0,0) = (((p2.y-p1.y)*(p3.z-p1.z))-((p2.z-p1.z)*(p3.y-p1.y)));
		coeff(0,1) = (((p2.z-p1.z)*(p3.x-p1.x))-((p2.x-p1.x)*(p3.z-p1.z)));
		coeff(0,2) = (((p2.x-p1.x)*(p3.y-p1.y))-((p2.y-p1.y)*(p3.x-p1.x)));
		coeff(0,3) = -(coeff(0,0)*p1.x+coeff(0,1)*p1.y+coeff(0,2)*p1.z);
   
        coeff(1,0) = p2.x-p1.x;
        coeff(1,1)=p2.y-p1.y;
        coeff(1,2)=p2.z-p1.z;
        coeff(1,3)=(p1.x*p1.x+p1.y*p1.y+p1.z*p1.z-(p2.x*p2.x+p2.y*p2.y+p2.z*p2.z))/2;

        coeff(2,0)=p2.x-p3.x;
        coeff(2,1)=p2.y-p3.y;
        coeff(2,2)=p2.z-p3.z;
        coeff(2,3)=(p3.x*p3.x+p3.y*p3.y+p3.z*p3.z-(p2.x*p2.x+p2.y*p2.y+p2.z*p2.z))/2;       

        // std::cout << "coeff: " << coeff << std::endl;

        PointT p0;
        PointT pR;
        p0=findSolution<pcl::PointXYZ>(coeff);
        p0 = PointT(-p0.x,-p0.y,-p0.z );
        // std::cout << "p0: " << p0 << std::endl;
        a=p1.x-p0.x;
        b=p1.y-p0.y;
        c=p1.z-p0.z;
        // std::cout << a << ":" << b << ":" << b << ":" << std::endl;
        double r=sqrt(a*a+b*b+c*c);
        // std::cout << "sph rad: " << sphereRadius << ", r: " << r << std::endl;
        if(sphereRadius>r){
            // std::cout<<"sphere O detected"<<std::endl;
            double distance_O=sqrt(sphereRadius*sphereRadius-r*r);
            pR.x=coeff(0,0)*distance_O+p0.x;
            pR.y=coeff(0,1)*distance_O+p0.y;
            pR.z=coeff(0,2)*distance_O+p0.z;
            for(int pt_cnt=0;pt_cnt<cloud->points.size();pt_cnt++)
            {
                if(pt_cnt!=idx1||pt_cnt!=idx2||pt_cnt!=idx3)
                {
                    a = cloud->points[pt_cnt].x-pR.x;
                    b = cloud->points[pt_cnt].y-pR.y;
                    c = cloud->points[pt_cnt].z-pR.z;
                    double dist = sqrt(a*a+b*b+c*c);
                    // If disttance is smaller than threshold count it as inlier
                    if(sphereRadius+disttanceThreshold>dist && sphereRadius-disttanceThreshold<dist )
                    {
                        inliers.insert(pt_cnt);
                    }
                }
            }
                if(inliers.size()>inliersResult.size())
                {
                    inliersResult.clear();
                    inliersResult = inliers;
                }
            pR.x=p0.x-coeff(0,0)*distance_O;
            pR.y=p0.y-coeff(0,1)*distance_O;
            pR.z=p0.z-coeff(0,2)*distance_O;
            for(int pt_cnt=0;pt_cnt<cloud->points.size();pt_cnt++)
            {
                if(pt_cnt!=idx1||pt_cnt!=idx2||pt_cnt!=idx3)
                {
                    a = cloud->points[pt_cnt].x-pR.x;
                    b = cloud->points[pt_cnt].y-pR.y;
                    c = cloud->points[pt_cnt].z-pR.z;
                    double dist = sqrt(a*a+b*b+c*c);
                    // If disttance is smaller than threshold count it as inlier
                    if(sphereRadius+disttanceThreshold>dist && sphereRadius-disttanceThreshold<dist)
                    {
                        inliers.insert(pt_cnt);
                    }
                }
            } 
                if(inliers.size()>inliersResult.size())
                {
                    inliersResult.clear();
                    inliersResult = inliers;
                    pT=pR;
                }                       
        }

		// Measure disttance between every point and fitted plane


		/*Store the temporary buffer if the size if more than previously idenfitied points */


	}

	// Segment the largest planar component from the remaining cloud
	if (inliersResult.size () == 0)
	{
	  std::cerr << "Could not estimate a sphere model for the given dataset." << std::endl;
	}
    // else
    //     std::cout << "plane coeffs are: a=" << a << ", b=" << b << ", c=" << c << ", d=" << d << std::endl;
	typename pcl::PointCloud<PointT>::Ptr cloudInliers(new pcl::PointCloud<PointT>());
	typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new pcl::PointCloud<PointT>());

	/*Copy the points from inputcloud in to cloudInliers if the indices is in inliersResult vector
	 * or else copy the point to cloudOutliers*/
	for(int index = 0; index < cloud->points.size(); index++)
	{
		PointT point = cloud->points[index];
		if(inliersResult.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}
	/*Create a pair using inlier and outlier points*/
	std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloudOutliers, cloudInliers);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "sphere segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;
    std::pair<PointT, std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>> allResult(pT, segResult);

    return allResult;
}