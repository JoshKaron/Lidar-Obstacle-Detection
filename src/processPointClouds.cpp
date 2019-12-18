// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"


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

    // down sample
    typename pcl::PointCloud<PointT>::Ptr cloudFiltered (new pcl::PointCloud<PointT>);
    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(filterRes, filterRes, filterRes);
    vg.filter(*cloudFiltered);

    // fit to region
    typename pcl::PointCloud<PointT>::Ptr cloudRegion (new pcl::PointCloud<PointT>);
    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloudFiltered);
    region.filter(*cloudRegion);

    // remove roof points
    std::vector<int> indicies;
    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1));
    roof.setInputCloud(cloudRegion);
    roof.filter(indicies);

    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    for(int idx : indicies)
        inliers->indices.push_back(idx);
    
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloudRegion);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloudRegion);


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRegion;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ExtractIndices(pcl::PointIndices::Ptr indices, typename pcl::PointCloud<PointT>::Ptr cloud)
{
    typename pcl::PointCloud<PointT>::Ptr extracted (new pcl::PointCloud<PointT>());

    for (int idx : indices->indices)
    {
        extracted->points.push_back(cloud->points[idx]);
    }

    return extracted;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr plane (new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr obs (new pcl::PointCloud<PointT>());
    
    for (int idx : inliers->indices)
    {
        plane->points.push_back(cloud->points[idx]);
    }
    
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    //extract.setNegative(false);
    //extract.filter(*plane);

    extract.setNegative(true);
    extract.filter(*obs);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obs, plane);
    
    return segResult;
}

template<typename PointT>
pcl::PointIndices::Ptr RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
    pcl::PointIndices::Ptr inliersResult {new pcl::PointIndices};
	srand(time(NULL));
	
	int n = cloud->points.size();

	for(int i = 0; i < maxIterations; i++)
	{
        pcl::PointIndices::Ptr tempResult {new pcl::PointIndices};

		// Randomly sample subset and fit line
		int rand1 = rand() % n;
		int rand2 = rand() % n;
		int rand3 = rand() % n;

		PointT p1 = cloud->points[rand1];
		PointT p2 = cloud->points[rand2];
		PointT p3 = cloud->points[rand3];

		// line represented by A, B, C
		double A = (p2.y-p1.y)*(p3.z-p1.z) - (p2.z-p1.z)*(p3.y-p1.y);
		double B = (p2.z-p1.z)*(p3.x-p1.x) - (p2.x-p1.x)*(p3.z-p1.z);
		double C = (p2.x-p1.x)*(p3.y-p1.y) - (p2.y-p1.y)*(p3.x-p1.x);
		double D = -(A*p1.x + B*p1.y + C*p1.z);
		double denom = sqrt(A*A + B*B + C*C);

		// Measure distance between every point and fitted line
		for(int j=0; j < n; j++)
		{
			PointT p = cloud->points[j];
			
			// If distance is smaller than threshold count it as inlier
			double dis = fabs(A*p.x + B*p.y + C*p.z + D) / denom;
			if(dis <= distanceTol)
			{
                tempResult->indices.push_back(j);
			}
		}

		if(tempResult->indices.size() > inliersResult->indices.size())
		{
			inliersResult = tempResult;
		}
	}
	
	return inliersResult;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	std::cout << "plane segmentation started with " << maxIterations << " itterations"<< std::endl;

    //pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    //pcl::ModelCoefficients::Ptr coef {new pcl::ModelCoefficients};
    //pcl::SACSegmentation<PointT> seg;
    //seg.setOptimizeCoefficients(true);
    //seg.setOptimizeCoefficients(false);
    //seg.setModelType(pcl::SACMODEL_PLANE);
    //seg.setMethodType(pcl::SAC_RMSAC);
    //seg.setMaxIterations(maxIterations);
    //seg.setDistanceThreshold(distanceThreshold);
    //seg.setInputCloud(cloud);
    //seg.segment(*inliers, *coef);

    pcl::PointIndices::Ptr inliers = RansacPlane<PointT>(cloud, maxIterations, distanceThreshold);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}


template<typename PointT>
void Proximity(int pointIdx, std::vector<int>& cluster,typename pcl::PointCloud<PointT>::Ptr cloud, std::vector<bool>& pointProcessed, KdTree* tree, float disTol)
{
	pointProcessed[pointIdx] = true;
	cluster.push_back(pointIdx);

	PointT point = cloud->points[pointIdx];
	std::vector<int> nearby = tree->search(point,disTol);
	for(int nearIdx : nearby)
	{
		if(pointProcessed[nearIdx] == true ) { continue; }

		Proximity<PointT>(nearIdx, cluster, cloud, pointProcessed, tree, disTol);
	}

}

template<typename PointT>
std::vector<std::vector<int>> euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, KdTree* tree, float distanceTol)
{

	// TODO: Fill out this function to return list of indices for each cluster
	std::vector<bool> pointProcessed;
	for(int i = 0; i < cloud->size(); i++) { pointProcessed.push_back(false); }

	std::vector<std::vector<int>> clusters;

	for(int i = 0; i < cloud->size(); i++)
	{
		if(pointProcessed[i] == false)
		{
			std::vector<int> cluster;
			Proximity<PointT>(i, cluster, cloud, pointProcessed, tree, distanceTol);
			clusters.push_back(cluster);
		}
	}
 
	return clusters;
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    //typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    //tree->setInputCloud(cloud);
    //std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    //std::vector<pcl::PointIndices> cluster_indicies;
    //pcl::EuclideanClusterExtraction<PointT> ec;
    //ec.setClusterTolerance(clusterTolerance);
    //ec.setMinClusterSize(minSize);
    //ec.setMaxClusterSize(maxSize);
    //ec.setSearchMethod(tree);
    //ec.setInputCloud(cloud);
    //ec.extract(cluster_indicies);
    //for(pcl::PointIndices indices : cluster_indicies)
    //{
    //    //clusters.push_back(ExtractIndices(indices, cloud));
    //    typename pcl::PointCloud<PointT>::Ptr cluster (new pcl::PointCloud<PointT>);
    //    for (int idx : indices.indices)
    //    {
    //        cluster->points.push_back(cloud->points[idx]);
    //    }
    //    cluster->width = cluster->points.size();
    //    cluster->height = 1;
    //    cluster->is_dense = true;
    //    
    //    clusters.push_back(cluster);
    //} 


    KdTree kdtree;
    kdtree.setInputCloud<PointT>(cloud);

    std::vector<std::vector<int>> clusterIndicies = euclideanCluster<PointT>(cloud, &kdtree, clusterTolerance);

    for(std::vector<int> indices : clusterIndicies)
    {
        if(indices.size() < minSize || indices.size() > maxSize) { continue; }
        
        typename pcl::PointCloud<PointT>::Ptr cluster (new pcl::PointCloud<PointT>);
        for (int idx : indices)
        {
            cluster->points.push_back(cloud->points[idx]);
        }
        cluster->width = cluster->points.size();
        cluster->height = 1;
        cluster->is_dense = true;
        
        clusters.push_back(cluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

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