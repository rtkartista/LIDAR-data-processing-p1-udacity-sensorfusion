// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"
#include "kdtree.h"
#include "render/render.h"

void CityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
    // filtering the input cloud by downsampling and cropping the region of interest
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessorI->FilterCloud(inputCloud, 0.275 , Eigen::Vector4f (-15, -5, -2, 1), Eigen::Vector4f ( 15, 7, 1, 1));

    // segmentation of the filtered cloud to separate highway from the obstacles
    std::unordered_set<int> inliers = pointProcessorI->Ransac(filterCloud, 30, 0.2);

	pcl::PointCloud<pcl::PointXYZI>::Ptr cloudInliers(new pcl::PointCloud<pcl::PointXYZI>());
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZI>());

	for(int index = 0; index < filterCloud->points.size(); index++)
	{
		if(inliers.count(index))
		{	
            cloudOutliers->points.push_back(filterCloud->points[index]);
        }
		else
        {
            cloudInliers->points.push_back(filterCloud->points[index]);
        }
	}

    //renderPointCloud(viewer,cloudInliers,"obstCloud",Color(1,0,0));
    renderPointCloud(viewer,cloudOutliers,"planeCloud",Color(0,1,0));

    // Finding data clusters representing obstacles in the segmented data
    KdTree* tree = new KdTree;
    std::vector<float> pt;

    for (int i=0; i<cloudInliers->points.size(); i++)
    { 
        pt = {cloudInliers->points[i].x, cloudInliers->points[i].y, cloudInliers->points[i].z};
        tree->insert(pt,i); 
    }

  	std::vector<std::vector<int>> clusters = pointProcessorI->euclideanCluster(cloudInliers, tree, .4, 8, 475);
  	
    // render box depicting roof
    int clusterId = 1;
    std::vector<Color> colors = {Color(1,0,0), Color(0,0,1), Color(1,1,0)};
    Box boxed;
    boxed.x_min = -1.25;
    boxed.y_min = -1.5;
    boxed.z_min = -1;
    boxed.x_max = 2.25;
    boxed.y_max = 1.5;
    boxed.z_max = -0.4;
    renderBox(viewer, boxed, 0, Color(1,1,1));

    // box fitting around clusters
    for(std::vector<int> clustert : clusters)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr clustercloud(new pcl::PointCloud<pcl::PointXYZI>());

        for(int index : clustert)
        {
            clustercloud->points.push_back(cloudInliers->points[index]);
        }
        clustercloud->width = clustercloud->points.size();
        clustercloud->height = 1;
        clustercloud->is_dense = true;

        renderPointCloud(viewer,clustercloud,"obstCloud"+std::to_string(clusterId),colors[clusterId]);
        
        Box box_draw = pointProcessorI->BoundingBox(clustercloud);
        //BoxQ box_draw = pointProcessorI->BoundingBox_R(clustercloud);
        renderBox(viewer, box_draw, clusterId);

        ++clusterId;
    }
}

//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}

int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    
    // Creating a ProcessPointCloud object to start obstacle detection
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    
    // Stream the PCD from stored files
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    
    // Creating a cloud object to load the streamed PCD
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;
    
    while (!viewer->wasStopped ())
    {
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and begin obstacle detection
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        CityBlock(viewer, pointProcessorI, inputCloudI);

        streamIterator++;
        if(streamIterator == stream.end())
            streamIterator = stream.begin();

        viewer->spinOnce ();
    }
}