// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}

pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}
/*
std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function
	float A, B, C, distance;
	float x1, x2, y1, y2;

	// For max iterations 
	for(int i = 0; i < maxIterations; i++)
	{
		std::unordered_set<int> inlier;
		// Randomly sample subset and fit line
		while(inlier.size() < 2)
		{
			inlier.insert(rand() % cloud->points.size());
		}

		// fitted line
		auto itr = inlier.begin();
		x1 = cloud->points[*itr].x;
		y1 = cloud->points[*itr].y;
		itr++;
		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;
		B = x2 - x1;
		A = y1 - y2;
		C = ( x1 * y2 ) - ( y1 * x2 );

		// Measure distance between every point and fitted line
		// If distance is smaller than threshold count it as inlier
		// no - 2 since we need those two lines too to be represented
		for (int j = 0; j < cloud->points.size(); j++)
		{
			distance = fabs(( A * cloud->points[j].x )+( B * cloud->points[j].y ) + C )/sqrt( A * A + B * B );
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

	// Return indicies of inliers from fitted line with most inliers
	return inliersResult;

}
*/
typename pcl::PointCloud<PointT>;
std::unordered_set<int> Ransac(pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function
	float A, B, C, D, distance;
	float x1, x2, x3, y1, y2, y3, z1, z2, z3;

	// For max iterations 
	for(int i = 0; i < maxIterations; i++)
	{
		std::unordered_set<int> inlier;
		// Randomly sample subset and fit line
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

	// Return indicies of inliers from fitted line with most inliers
	return inliersResult;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	// pcl::PointCloud<PointT>::Ptr cloud = CreateData();

	// For the new ransac implementation
	pcl::PointCloud<PointT>::Ptr cloud = CreateData3D();


	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac(cloud, 50, .5);

	pcl::PointCloud<PointT>::Ptr cloudInliers(new pcl::PointCloud<PointT>());
	pcl::PointCloud<PointT>::Ptr cloudOutliers(new pcl::PointCloud<PointT>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		PointT point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
