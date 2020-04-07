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

void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZ>* pointProcessor, const pcl::PointCloud<pcl::PointXYZ>::Ptr& inputCloud)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    //renderRays(viewer, lidar->position, lidarCloud);
    //renderPointCloud(viewer, lidarCloud, "LidarCloud", Color(1,1,1));
    // TODO:: Create point processor
    std::pair<boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>, boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>> segmentedPair = pointProcessor->SegmentPlaneCustom(inputCloud, 100, 0.2);
    
    // Render plane and object clouds separately:
    //renderPointCloud(viewer, segmentedPair.first, "planeCloud", Color(0,1,0));
    renderPointCloud(viewer, segmentedPair.second, "obstacleCloud", Color(0,0,1));

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
    clusters = pointProcessor->Clustering(segmentedPair.second, 1.0, 0.1, 10.0);
    cout << "Total number of detected clusters: " << clusters.size() << endl;

    // Render all clusters:
    for (size_t clusterId=0; clusterId<clusters.size(); ++clusterId)
    {
        BoxQ boxq = pointProcessor->PCABoundingBox(clusters[clusterId]);
        renderBox(viewer, boxq, clusterId, Color(1,1,1), 0.5);
    }
}

template<typename PointT> 
void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<PointT>* pointProcessor, const typename pcl::PointCloud<PointT>::Ptr& inputCloud)
{
    typename pcl::PointCloud<PointT>::Ptr refinedCloud(new pcl::PointCloud<PointT>() );
    refinedCloud = pointProcessor->FilterCloud(inputCloud, 1.0, Eigen::Vector4f(-15,-15,-15,1), Eigen::Vector4f(15,15,15,1));
    std::pair<boost::shared_ptr<pcl::PointCloud<PointT>>, boost::shared_ptr<pcl::PointCloud<PointT>>> segmentedPair = pointProcessor->SegmentPlaneCustom(refinedCloud, 100, 0.2);
    renderPointCloud(viewer, segmentedPair.second, "inputCloud");

    /*std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters;
    clusters = pointProcessor.Clustering(segmentedPair.second, 1.0, 1.0, 10.0);
    cout << "Total number of detected clusters: " << clusters.size() << endl;

    // Render all clusters:
    for (size_t clusterId=0; clusterId<clusters.size(); ++clusterId)
    {
        Box box = pointProcessor.BoundingBox(clusters[clusterId]);
        renderBox(viewer, box, clusterId, Color(1,1,1), 0.5);
    }*/
}




template 
<typename PointT>
class SceneWrapper 
{
    ProcessPointClouds<PointT>* pointProcessor;
    typename pcl::PointCloud<PointT>::Ptr inputCloud;
    std::vector<boost::filesystem::path> stream;
    pcl::visualization::PCLVisualizer::Ptr viewer;
    bool streamingFlag;
    CameraAngle setAngle;

    public:
        SceneWrapper(std::string path, bool streamingFlag) : viewer( new pcl::visualization::PCLVisualizer ("3D Viewer"))
        {
            setAngle = XY;
            initCamera(setAngle, viewer);
            pointProcessor = new ProcessPointClouds<PointT>();
            stream = pointProcessor->streamPcd(path);
            
            this->streamingFlag = streamingFlag;
        }

        SceneWrapper() : viewer( new pcl::visualization::PCLVisualizer ("3D Viewer"))
        {
            setAngle = XY;
            initCamera(setAngle, viewer);
            pointProcessor = new ProcessPointClouds<PointT>();
            bool renderScene = true;
            std::vector<Car> cars = initHighway(renderScene, viewer);
            Lidar* lidar = new Lidar(cars, 0);
            inputCloud = lidar->scan();
        }

        ~SceneWrapper() {}

        void Render()
        {
            auto streamIterator = stream.begin();
            inputCloud = pointProcessor->loadPcd( (*streamIterator).string() );
            cityBlock<PointT>(viewer, pointProcessor, inputCloud);

            while (!viewer->wasStopped ())
            {
                if (streamingFlag==true)
                {
                    viewer->removeAllPointClouds();
                    viewer->removeAllShapes();
                    streamIterator++;
                    if (streamIterator == stream.end() ) streamIterator = stream.begin();
                    inputCloud = pointProcessor->loadPcd( (*streamIterator).string() );
                    cityBlock<PointT>(viewer, pointProcessor, inputCloud);
                }

                viewer->spinOnce ();
            }
        }
};



enum MODE {
    STATIC_HIGHWAY,
    STATIC_CITYBLOCK,
    DYNAMIC_CITYBLOCK
};


int main (int argc, char** argv)
{
    MODE mode = MODE(1);

   
    if (mode == STATIC_HIGHWAY)
    {
        std::cout << "Starting enviroment static highway" << std::endl;
        SceneWrapper<pcl::PointXYZ> scene();
        //scene.Render();
    }
    else if (mode == STATIC_CITYBLOCK)
    {
        std::cout << "Starting enviroment static city block" << std::endl;
        SceneWrapper<pcl::PointXYZI> scene("../src/sensors/data/pcd/data_1", false);
        scene.Render();
    }
    else if (mode == DYNAMIC_CITYBLOCK)
    {
        std::cout << "Starting enviroment dynamic cityblock" << std::endl;
        SceneWrapper<pcl::PointXYZI> scene("../src/sensors/data/pcd/data_1", true);
        scene.Render();
    }
}
