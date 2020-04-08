/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway()//bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
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

    /*if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }*/

    return cars;
}



template<typename PointT>
class SceneWrapper 
{
    ProcessPointClouds<PointT>* pointProcessor;
    typename pcl::PointCloud<PointT>::Ptr inputCloud;
    std::vector<boost::filesystem::path> stream;
    pcl::visualization::PCLVisualizer::Ptr viewer;
    bool streamingFlag;
    CameraAngle setAngle;
    std::string path;
    std::vector<boost::filesystem::path>::iterator streamIterator;

    private:
        //setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
        void initCamera()
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

        void render(typename pcl::PointCloud<PointT>::Ptr& inputCloud)
        {
            typename pcl::PointCloud<PointT>::Ptr refinedCloud(new pcl::PointCloud<PointT>() );
            refinedCloud = pointProcessor->FilterCloud(inputCloud, 1.0, Eigen::Vector4f(-15,-6,-2,1), Eigen::Vector4f(15,6,15,1));
            std::pair<boost::shared_ptr<pcl::PointCloud<PointT>>, boost::shared_ptr<pcl::PointCloud<PointT>>> segmentedPair = pointProcessor->SegmentPlaneCustom(refinedCloud, 100, 0.2);
            renderPointCloud(viewer, segmentedPair.second, "inputCloud");

            std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
            clusters = pointProcessor->Clustering(segmentedPair.second, 1.0, 1.0, 10.0);
            cout << "Total number of detected clusters: " << clusters.size() << endl;
            
            // Render all clusters:
            for (size_t clusterId=0; clusterId<clusters.size(); ++clusterId)
            {
                Box box = pointProcessor->BoundingBox(clusters[clusterId]);
                renderBox(viewer, box, clusterId, Color(1,1,1), 0.5);
            }
        }

        typename pcl::PointCloud<PointT>::Ptr getCloudPointsFromFile()
        {
            inputCloud = pointProcessor->loadPcd( path );
            return inputCloud;
        }

        typename pcl::PointCloud<PointT>::Ptr getCloudPointsFromDirectory()
        {
            inputCloud = pointProcessor->loadPcd( (*streamIterator).string() );
            streamIterator++;
            if (streamIterator == stream.end() ) streamIterator = stream.begin();
            return inputCloud;
        }

        typename pcl::PointCloud<PointT>::Ptr getCloudPointsFixed()
        {
            return inputCloud;
        }

        typename pcl::PointCloud<PointT>::Ptr (SceneWrapper::*getCloudPoints)(); // Function pointer

    public:
        SceneWrapper(typename pcl::PointCloud<PointT>::Ptr inputCloud) : viewer( new pcl::visualization::PCLVisualizer ("3D Viewer"))
        {
            setAngle = XY;
            initCamera();
            pointProcessor = new ProcessPointClouds<PointT>();
            bool renderScene = false;

            this->streamingFlag = false;            
            this->inputCloud = inputCloud;
            this->getCloudPoints = &SceneWrapper::getCloudPointsFixed;
        }

        SceneWrapper(std::string path) : viewer( new pcl::visualization::PCLVisualizer ("3D Viewer"))
        {
            setAngle = XY;
            initCamera();
            pointProcessor = new ProcessPointClouds<PointT>();
            bool renderScene = true;

            if (boost::filesystem::is_directory(path))
            {
                streamingFlag = true;
                stream = pointProcessor->streamPcd(path);
                streamIterator = stream.begin();
                this->getCloudPoints = &SceneWrapper::getCloudPointsFromDirectory;
            }
            else if (boost::filesystem::is_regular_file(path))
            {
                streamingFlag = false;
                this->path = path;
                this->getCloudPoints = &SceneWrapper::getCloudPointsFromFile;
            }

        }

        ~SceneWrapper() {}

        void Spin()
        {
            inputCloud = (this->*getCloudPoints)();
            render(inputCloud);
            
            while (!viewer->wasStopped ())
            {
                if (streamingFlag==true)
                {
                    viewer->removeAllPointClouds();
                    viewer->removeAllShapes();
                    inputCloud = (this->*getCloudPoints)();
                    render(inputCloud);
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

    // Simplest possible environment selector - no proof for valid values
    if (argc == 2) {
        mode = MODE( boost::lexical_cast<int>(argv[1][0]) );
    }

    if (mode == STATIC_HIGHWAY)
    {
        std::cout << "Starting enviroment static highway" << std::endl;
        std::vector<Car> cars = initHighway();
        Lidar* lidar = new Lidar(cars, 0);
        SceneWrapper<pcl::PointXYZ> scene(lidar->scan());
        scene.Spin();
    }
    else if (mode == STATIC_CITYBLOCK)
    {
        std::cout << "Starting enviroment static city block" << std::endl;
        SceneWrapper<pcl::PointXYZI> scene("../src/sensors/data/pcd/data_1/0000000000.pcd");
        scene.Spin();
    }
    else if (mode == DYNAMIC_CITYBLOCK)
    {
        std::cout << "Starting enviroment dynamic cityblock" << std::endl;
        SceneWrapper<pcl::PointXYZI> scene("../src/sensors/data/pcd/data_1");
        scene.Spin();
    }
}
