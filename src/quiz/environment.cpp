// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

/*std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
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


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
    // cars from init highway and groundslope = 0
    // heap object
    Lidar* lidar_object = new Lidar(cars, 0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pt_clouds = lidar_object->scan(); 
    // renderRays(viewer, lidar_object->position, pt_clouds);
    // clearRays(viewer);
    renderPointCloud(viewer, pt_clouds, "pts");
    // TODO:: Create point processor
    // ProcessPointCLouds<pcl::PointXYZ> pPC_object;
    ProcessPointClouds<pcl::PointXYZ>* pPC_object = new ProcessPointClouds<pcl::PointXYZ>();
    
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pPC_object->SegmentPlane(pt_clouds, 100, 0.2);
    renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(1,0,0));
    renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));
    
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pPC_object->Clustering(segmentCloud.first, 1.0, 3, 30);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};

    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        pPC_object->numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);
        Box box_draw = pPC_object->BoundingBox(cluster);
        //BoxQ box_draw = pPC_object->BoundingBox(cluster);
        renderBox(viewer, box_draw, clusterId);
        ++clusterId;
    }
}*/

void CityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------

    // filtering
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessorI->FilterCloud(inputCloud, .3 , Eigen::Vector4f (-13, -5, -5, 1), Eigen::Vector4f ( 13, 7, 10, 1));
    
    // segmetation    
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->SegmentPlane(filterCloud, 100, 0.3);
    renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(1,0,0));
    renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));
    
    // clustering
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(segmentCloud.first, .5, 10, 500);

    int clusterId = 1;
    std::vector<Color> colors = {Color(1,0,0), Color(0,0,1), Color(1,1,0)};
        Box boxed;
        boxed.x_min = -1.5;
        boxed.y_min = -1.5;
        boxed.z_min = -1;
        boxed.x_max = 2.6;
        boxed.y_max = 1.5;
        boxed.z_max = -0.4;
        renderBox(viewer, boxed, 0, Color(1,1,1));
    // box fitting
    for(pcl::PointCloud<pcl::PointXYZI>::Ptr clustert : cloudClusters)
    {
        std::cout << "cluster size ";
        pointProcessorI->numPoints(clustert);
        //renderPointCloud(viewer,clustert,"obstCloud"+std::to_string(clusterId),colors[clusterId]);
        Box box_draw = pointProcessorI->BoundingBox(clustert);
        //BoxQ box_draw = pointProcessorI->BoundingBox(cluster);
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
    //simpleHighway(viewer);
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;
    
    while (!viewer->wasStopped ())
    {

    // Clear viewer
    viewer->removeAllPointClouds();
    viewer->removeAllShapes();

    // Load pcd and run obstacle detection process
    inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
    CityBlock(viewer, pointProcessorI, inputCloudI);

    streamIterator++;
    if(streamIterator == stream.end())
        streamIterator = stream.begin();

    viewer->spinOnce ();
    }
}