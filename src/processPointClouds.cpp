// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr& cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    // downsapling the data using voxelgrid with leaf size of .2m
    pcl::VoxelGrid<PointT> vg;
    typename pcl::PointCloud<PointT>::Ptr cloudFiltered (new pcl::PointCloud<PointT>);
    vg.setInputCloud(cloud);
    vg.setLeafSize(filterRes, filterRes, filterRes);
    vg.filter(*cloudFiltered);

    // setting the region of interest
    typename pcl::PointCloud<PointT>::Ptr cloudRegion (new pcl::PointCloud<PointT>);

    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloudFiltered);
    region.filter(*cloudRegion);

    // storing roof points
    std::vector<int> indices;
    
    pcl::CropBox<PointT> roof(true);
    roof.setMin({-1.5,-1.7,-1,1});
    roof.setMax({2.6,1.7,-0.4,1});
    roof.setInputCloud(cloudRegion);
    roof.filter(indices);
    
    // exctracting the non roof points
    pcl::PointIndices::Ptr inliners {new pcl::PointIndices};
    for (int point: indices)
        inliners->indices.push_back(point);
    
    pcl::ExtractIndices<PointT> ex;
    ex.setInputCloud(cloudRegion);
    ex.setIndices(inliners);
    ex.setNegative(true);
    ex.filter(*cloudRegion);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRegion;

}

template<typename PointT>
std::unordered_set<int> ProcessPointClouds<PointT>::Ransac(typename pcl::PointCloud<PointT>::Ptr& cloud, int maxIterations, float distanceTol)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function
	float A, B, C, D, distance;
	float x1, x2, x3, y1, y2, y3, z1, z2, z3;

	// For max iterations 
	for(int i = 0; i < maxIterations; i++)
	{
		std::unordered_set<int> inlier;
		// Randomly sample subset and fit plane
		while(inlier.size() < 3)
		{
			inlier.insert(rand() % cloud->points.size());
		}

		// fitted plane
		auto itr = inlier.begin();
		x1 = cloud->points[*itr].x;
		y1 = cloud->points[*itr].y;
		z1 = cloud->points[*itr].z;
		itr++;
		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;
		z2 = cloud->points[*itr].z;
		itr++;
		x3 = cloud->points[*itr].x;
		y3 = cloud->points[*itr].y;
		z3 = cloud->points[*itr].z;

		A = (y2-y1)*(z3-z1) - (z2-z1)*(y3-y1);
		B = (z2-z1)*(x3-x1) - (x2-x1)*(z3-z1);
		C = (x2-x1)*(y3-y1) - (y2-y1)*(x3-x1);
		D = - ( A * x1 + B * y1 + C * z1 );

		// Measure distance between every point and fitted line
		// If distance is smaller than threshold count it as inlier
		// no - 3 since we need those two lines too to be represented
		for (int j = 0; j < cloud->points.size(); j++)
		{
			distance = fabs(( A * cloud->points[j].x )+( B * cloud->points[j].y ) + ( C * cloud->points[j].z ) + D )/sqrt( A * A + B * B + C * C);
			if(distance <= distanceTol)
			{
				inlier.insert(j);
			}		
		}

		if (inlier.size() > inliersResult.size())
		{
			inliersResult = inlier;
		}
		
	}

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;


	// Return indicies of inliers from fitted line with most inliers
	return inliersResult;

}

template<typename PointT>
void ProcessPointClouds<PointT>::proximity(int it, typename pcl::PointCloud<PointT>::Ptr& cloud, std::vector<int>& cluster_p, std::vector<bool> &processed, KdTree* tree, float distanceTol)
{
	processed[it] = true;
	cluster_p.push_back(it); 

    std::vector<float> pt = {cloud->points[it].x, cloud->points[it].y, cloud->points[it].z};
	std::vector<int> nearest = tree->search(pt, distanceTol);

	for(int id : nearest)
	{
		// if the points nearest to the points[it] are not processed add the points in cluster
		if(processed[id] == false)
			proximity(id, cloud, cluster_p, processed, tree, distanceTol);
	}
}

template<typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanCluster(typename pcl::PointCloud<PointT>::Ptr& cloud, KdTree* tree, float distanceTol,int minSize, int maxSize)
{
  	auto startTime = std::chrono::steady_clock::now();

	// TODO: Fill out this function to return list of indices for each cluster

	std::vector<std::vector<int>> clusters;
	std::vector<bool> processed(cloud->points.size(),false);

    for (int i = 0; i < cloud->points.size(); i++) 
    {
        if (processed[i]) { continue; }

        std::vector<int> cluster;  // Create cluster

        proximity(i, cloud, cluster, processed, tree, distanceTol);
        if(cluster.size()>=minSize && cluster.size()<= maxSize)
        {
            clusters.push_back(cluster);
        }
    }
    delete tree;
    tree = nullptr;
    
    auto endTime = std::chrono::steady_clock::now();
  	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  	std::cout << "clusters found " << clusters.size() << " and took " << elapsedTime.count() << " milliseconds" << std::endl;

	return clusters;
}

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
BoxQ ProcessPointClouds<PointT>::BoundingBox_R(typename pcl::PointCloud<PointT>::Ptr cluster)
{
    BoxQ box;

    // Use PCA and get the components
    typename pcl::PointCloud<PointT>::Ptr cloudPCAprojection(new pcl::PointCloud<PointT>);
    pcl::PCA<PointT> pca; 

    // Setting the z components of the point cloud data to 0 to rotate the boxes in the XY plane
    typename pcl::PointCloud<PointT>::Ptr cluster2(new pcl::PointCloud<PointT>);;
    for(int i=0; i<cluster->points.size();i++)
    {
        cluster2->points.push_back(cluster->points[i]);
        cluster2->points[i].z = 0;
    }
  
    pca.setInputCloud(cluster2);
    pca.project(*cluster2, *cloudPCAprojection);

    // Transform the original cloud to the origin where the principal components correspond to the axes.
    Eigen::Vector4f pcaCentroid;
    pcl::compute3DCentroid(*cluster, pcaCentroid);
    Eigen::Matrix3f eigenVectorsPCA = pca.getEigenVectors();

    Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
    projectionTransform.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();
    projectionTransform.block<3, 1>(0, 3) = -1.f * (projectionTransform.block<3, 3>(0, 0) * pcaCentroid.head<3>());
    typename pcl::PointCloud<PointT>::Ptr cloudPointsProjected(
        new pcl::PointCloud<PointT>);
    pcl::transformPointCloud(*cluster, *cloudPointsProjected,
                            projectionTransform);

    // Get the minimum and maximum points of the transformed cloud.
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
     Eigen::Vector3f meanDiagonal =
        0.5f * (maxPoint.getVector3fMap() + minPoint.getVector3fMap());

    // Final Transform
     Eigen::Quaternionf bboxQuaternion(
        eigenVectorsPCA);  //Quaternions 
     Eigen::Vector3f bboxTransform =
        eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();

    // Set parameters and return
    box.bboxTransform = bboxTransform;
    box.bboxQuaternion = bboxQuaternion;
    // FInding the height of the box enclosing the cluster
    PointT minP, maxP;
    pcl::getMinMax3D(*cluster, minP, maxP);

    box.cube_length = maxPoint.x - minPoint.x;
    box.cube_width = maxPoint.y - minPoint.y;
    box.cube_height = maxP.z - minP.z;
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