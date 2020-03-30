/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

#include <random>

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

// TODO: Fill in this function
std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	std::srand ( unsigned ( std::time(0) ) );

	// Just use random_shuffle, as the correct sequence of the points doesn't matter anyway.
	std::vector<int> randVec;
	for (uint i=0; i < cloud->points.size(); ++i) randVec.push_back(i);

	uint size = cloud->points.size();
	pcl::PointXYZ pt1, pt2;

	// For max iterations 
	for (int i = 0; i<maxIterations; ++i)
	{
		// Randomly sample subset and fit line
		// Shuffle and select the first two points
		std::random_shuffle(randVec.begin(), randVec.end());
		pt1 = cloud->points[randVec[0]];
		pt2 = cloud->points[randVec[1]];

		// For debugging purposses only:
		//cout << "Point " << randVec[0] << ": " << pt1.x << " , " << pt1.y << endl;
		//cout << "Point " << randVec[1] << ": " << pt2.x << " , " << pt2.y << endl << endl;

		float A = pt1.y - pt2.y;
		float B = pt2.x - pt1.x;
		float C = pt1.x*pt2.y - pt2.x*pt1.y;
		float denominator = sqrt(A*A + B*B);
		float dist = 0;

		std::unordered_set<int> inliersTemp;

		// Measure distance between every point and fitted line
		// If distance is smaller than threshold count it as inlier
		for (int i=0; i<size; i++)
		{
			uint index = randVec[i];
			pcl::PointXYZ pt = cloud->points[index];
			dist = abs(A*pt.x + B*pt.y + C)/denominator;
			if (dist < distanceTol)
				inliersTemp.insert(index);
		}

		// Return indicies of inliers from fitted line with most inliers
		if (inliersTemp.size() > inliersResult.size())
			inliersResult = inliersTemp;
	}
	
	return inliersResult;
}


int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac(cloud, 100, 1.4);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
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



// Maybe useful for gist:
/*std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	std::random_device rd;
	std::mt19937 gen(rd());
	std::bernoulli_distribution b;
	
	// TODO: Fill in this function
	std::cout << "Size of set: " << cloud->points.size() << std::endl;

	// For max iterations 
	for (int i = 0; i<maxIterations; ++i)
	{
		//std::cout << b(gen) << std::endl;

		// Randomly sample subset and fit line
		std::vector<pcl::PointXYZ> subSample;
		for (auto pt : cloud->points)
			if (b(gen))	subSample.push_back(pt);
		
		// Calculate meanX and meanY of subsample;
		float meanX = 0, meanY = 0;
		
		for (auto pt : subSample)
		{
			meanX += pt.x;
			meanY += pt.y;
		}
		//std::cout << meanX << " , " << meanY << std::endl;

		float numerator = 0;
		float denumerator = 0;

		for (auto i : subSample)
		{
			numerator += (i.x - meanX)*(i.y-meanY);
			denumerator += pow(i.x - meanX, 2);
		}
		float m = numerator/denumerator;
		std::cout << "m = " << m << std::endl; 
		std::cout << "b = " << meanY-m*meanX << std::endl; 

	}

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with most inliers
	
	return inliersResult;

}*/