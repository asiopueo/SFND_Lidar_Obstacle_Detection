/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

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


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
    Lidar* lidar = new Lidar(cars, 0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr lidarCloud = lidar->scan();
    //renderRays(viewer, lidar->position, lidarCloud);
    //renderPointCloud(viewer, lidarCloud, "LidarCloud", Color(1,1,1));
    // TODO:: Create point processor
    ProcessPointClouds<pcl::PointXYZ> processPointClouds;

    //std::pair<boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>, boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>> segmentedPair = processPointClouds.SegmentPlane(lidarCloud, 100, 0.2);
    std::pair<boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>, boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>> segmentedPair = processPointClouds.SegmentPlaneCustom(lidarCloud, 100, 0.2);
    // Render plane and object clouds separately:
    //renderPointCloud(viewer, segmentedPair.first, "planeCloud", Color(0,1,0));
    renderPointCloud(viewer, segmentedPair.second, "obstacleCloud", Color(0,0,1));

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
    clusters = processPointClouds.Clustering(segmentedPair.second, 1.0, 0.1, 10.0);
    cout << "Total number of detected clusters: " << clusters.size() << endl;

    // Render all clusters:
    for (size_t clusterId=0; clusterId<clusters.size(); ++clusterId)
    {
        Box box = processPointClouds.BoundingBox(clusters[clusterId]);
        renderBox(viewer, box, clusterId, Color(1,1,1), 0.5);
    }
}


void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    ProcessPointClouds<pcl::PointXYZI> pointProcessor;
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessor.loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
    pcl::PointCloud<pcl::PointXYZI>::Ptr refinedCloud(new pcl::PointCloud<pcl::PointXYZI>() );
    
    refinedCloud = pointProcessor.FilterCloud(inputCloud, 1.0, Eigen::Vector4f(-15,-15,-15,1), Eigen::Vector4f(15,15,15,1));
    renderPointCloud(viewer, refinedCloud, "inputCloud");

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
    cityBlock(viewer);

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce ();
    } 
}