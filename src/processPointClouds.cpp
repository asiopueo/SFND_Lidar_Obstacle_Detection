// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include "kdtree.h"
#include <unordered_set>

//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // Voxel reduction:    
    typename pcl::PointCloud<PointT>::Ptr voxelCloud( new pcl::PointCloud<PointT>() );
    pcl::VoxelGrid<PointT> voxelFilter;
    voxelFilter.setInputCloud(cloud);
    const float leafSize = 0.2;
    voxelFilter.setLeafSize(leafSize, leafSize, leafSize);
    voxelFilter.filter(*voxelCloud);

    // Crop outside region:
    typename pcl::PointCloud<PointT>::Ptr regionCloud( new pcl::PointCloud<PointT>() );
    pcl::CropBox<PointT> cropRegion;
    cropRegion.setInputCloud(voxelCloud);
    cropRegion.setMin(minPoint);
    cropRegion.setMax(maxPoint);
    cropRegion.filter(*regionCloud);

    // Crop car roof (slightly more difficult, as there isn't a method for cropping inside a box):
    std::vector<int> indices;
    const float minRoof_x = -2.;
    const float maxRoof_x = 3.5;

    const float minRoof_y = -2.;
    const float maxRoof_y = 2.;

    const float maxRoof_z = 2.;
    const float minRoof_z = -2.5;

    pcl::CropBox<PointT> cropRoof( true );
    cropRoof.setInputCloud(regionCloud);
    cropRoof.setMin(Eigen::Vector4f(minRoof_x, minRoof_y, minRoof_z, 1.));
    cropRoof.setMax(Eigen::Vector4f(maxRoof_x, maxRoof_y, maxRoof_z, 1.));
    cropRoof.filter(indices);

    pcl::PointIndices::Ptr roofIndices( new pcl::PointIndices() );
    for (auto index : indices)
        roofIndices->indices.push_back(index);

    typename pcl::PointCloud<PointT>::Ptr refinedCloud( new pcl::PointCloud<PointT>() );
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(regionCloud);
    extract.setIndices(roofIndices);
    extract.setNegative( true );
    extract.filter(*refinedCloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return refinedCloud;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
    // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr planeCloud(new pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr obstacleCloud(new pcl::PointCloud<PointT>);
    
    for (auto index : inliers->indices)
        planeCloud->points.push_back(cloud->points[index]);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obstacleCloud);


    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacleCloud, planeCloud);
    return segResult;
}

// Alternative version of the SegmentPlane()-method with custom RANSAC-implementation
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlaneCustom(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    std::unordered_set<int> inliers;
	std::srand ( unsigned ( std::time(0) ) );

	// Just use random_shuffle, as the correct sequence of the points doesn't matter anyway.
	std::vector<int> randVec;
	for (uint i=0; i < cloud->points.size(); ++i) randVec.push_back(i);

	uint size = cloud->points.size();
	PointT pt1, pt2, pt3;

	// For max iterations 
	for (int i = 0; i<maxIterations; ++i)
	{
		// Randomly sample subset and fit line
		// Shuffle and select the first two points
		std::random_shuffle(randVec.begin(), randVec.end());
		pt1 = cloud->points[randVec[0]];
		pt2 = cloud->points[randVec[1]];
		pt3 = cloud->points[randVec[2]];

		float A = (pt2.y-pt1.y)*(pt3.z-pt1.z)-(pt2.z-pt1.z)*(pt3.y-pt1.y);
		float B = (pt2.z-pt1.z)*(pt3.x-pt1.x)-(pt2.x-pt1.x)*(pt3.z-pt1.z);
		float C = (pt2.x-pt1.x)*(pt3.y-pt1.y)-(pt2.y-pt1.y)*(pt3.x-pt1.x);
		float D = -(A*pt1.x+B*pt1.y+C*pt1.z);
		float denominator = sqrt(A*A + B*B + C*C);
		float dist = 0;

		std::unordered_set<int> inliersTemp;

		// Measure distance between every point and fitted line
		// If distance is smaller than threshold count it as inlier
		for (int i=0; i<size; i++)
		{
			uint index = randVec[i];
			PointT pt = cloud->points[index];
			dist = abs(A*pt.x + B*pt.y + C*pt.z + D)/denominator;
			if (dist < distanceThreshold)
				inliersTemp.insert(index);
		}

		// Return indicies of inliers from fitted line with most inliers
		if (inliersTemp.size() > inliers.size())
			inliers = inliersTemp;
	}

    // Prepare two genuine point clouds from original cloud
    typename pcl::PointCloud<PointT>::Ptr cloudInliers(new pcl::PointCloud<PointT>());
	typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new pcl::PointCloud<PointT>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		PointT point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloudInliers, cloudOutliers);
    return segResult;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in this function to find inliers for the cloud.
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::SACSegmentation<PointT> seg;

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() ==0)
        std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
    else
        std::cout << "inliers->indices.size(): " << inliers->indices.size() << std::endl; // For debugging purposes only. Remove this later.

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers, cloud);
    return segResult;
}


template<typename PointT>
void clusterHelper(typename pcl::PointCloud<PointT>::Ptr& cloud, std::unordered_set<int>& processedIndices, const int target, std::vector<int>& cluster, KdTree<PointT>* tree, float distanceTol)
{
	processedIndices.insert(target);
	cluster.push_back(target);
	
	std::vector<int> nbrPoints = tree->search(cloud->points[target], distanceTol);
	for (int nbrPoint : nbrPoints)
		if (!processedIndices.count(nbrPoint))
			clusterHelper(cloud, processedIndices, nbrPoint, cluster, tree, distanceTol);
}


// TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, const float clusterTolerance, const int minSize, const int maxSize, const float minDimension, const float maxDimension)
{
    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
	std::unordered_set<int> processedIndices;
 
    // Initialize Kd-Tree:
    KdTree<PointT>* tree = new KdTree<PointT>();
    for (size_t i=0; i<cloud->points.size(); ++i)
        tree->insert(cloud->points[i], i);

	for (size_t index = 0; index < cloud->points.size(); index++)
	{
		if (!processedIndices.count(index))
		{
			// Create cluster
			std::vector<int> cluster;
            typename pcl::PointCloud<PointT>::Ptr clusterCloud(new pcl::PointCloud<PointT>);
			clusterHelper(cloud, processedIndices, index, cluster, tree, clusterTolerance);
			
            for (size_t i : cluster)
            {
                PointT point = cloud->points[i];
                clusterCloud->points.push_back(point);
            }

            // Filtering via the number of points of the cluster:
            if (minSize < clusterCloud->points.size() && clusterCloud->points.size() < maxSize) {

                // Filtering via the absolute dimensions of the point cloud:
                PointT minPoint, maxPoint;
                pcl::getMinMax3D(*clusterCloud, minPoint, maxPoint);
                float max = std::max({maxPoint.x-minPoint.x, maxPoint.y-minPoint.y, maxPoint.z-minPoint.z });
                float min = std::min({maxPoint.x-minPoint.x, maxPoint.y-minPoint.y, maxPoint.z-minPoint.z });
                if (minDimension < max && max < maxDimension)
                    clusters.push_back(clusterCloud);
            }
		}
	}

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "Clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

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
BoxQ ProcessPointClouds<PointT>::PCABoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{
    // Computation of principal directions:
    Eigen::Vector4f pcaCentroid;
    pcl::compute3DCentroid(*cluster, pcaCentroid);

    Eigen::Matrix3f covariance;
    pcl::computeCovarianceMatrixNormalized(*cluster, pcaCentroid, covariance);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigenSolver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenVectors = eigenSolver.eigenvectors();
    eigenVectors.col(2) = eigenVectors.col(0).cross(eigenVectors.col(1));

    Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
    projectionTransform.block<3,3>(0,0) = eigenVectors.transpose();
    projectionTransform.block<3,1>(0,3) = -1.0 * projectionTransform.block<3,3>(0, 0) * pcaCentroid.head<3>();
    typename pcl::PointCloud<PointT>::Ptr cloudPointsProjected (new pcl::PointCloud<PointT>);
    pcl::transformPointCloud(*cluster, *cloudPointsProjected, projectionTransform);
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
    const Eigen::Vector3f meanDiagonal = 0.5 * (maxPoint.getVector3fMap() + minPoint.getVector3fMap());

    const Eigen::Quaternionf bboxQuaternion(eigenVectors);
    const Eigen::Vector3f bboxTransform = eigenVectors * meanDiagonal + pcaCentroid.head<3>();

    BoxQ boxQ{bboxTransform, bboxQuaternion, maxPoint.x-minPoint.x, maxPoint.y-minPoint.y, maxPoint.z-minPoint.z};
    
    return boxQ;
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