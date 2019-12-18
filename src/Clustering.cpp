#include "Clustering.h"

//constructor:
template<typename PointT>
Clustering<PointT>::Clustering() {}


//de-constructor:
template<typename PointT>
Clustering<PointT>::~Clustering() {}

template<typename PointT>
void proximity(int pointIdx, std::vector<int>& cluster,typename pcl::PointCloud<PointT>::Ptr cloud, std::vector<bool>& pointProcessed, KdTree* tree, float disTol)
{
	pointProcessed[pointIdx] = true;
	cluster.push_back(pointIdx);

	PointT point = cloud->points[pointIdx];
	std::vector<int> nearby = tree->search(point,disTol);
	for(int nearIdx : nearby)
	{
		if(pointProcessed[nearIdx] == true ) { continue; }

		proximity<PointT>(nearIdx, cluster, cloud, pointProcessed, tree, disTol);
	}

}

template<typename PointT>
std::vector<std::vector<int>> euclideanClustering(typename pcl::PointCloud<PointT>::Ptr cloud, KdTree* tree, float distanceTol)
{
	std::vector<bool> pointProcessed;
	for(int i = 0; i < cloud->size(); i++) { pointProcessed.push_back(false); }

	std::vector<std::vector<int>> clusters;

	for(int i = 0; i < cloud->size(); i++)
	{
		if(pointProcessed[i] == false)
		{
			std::vector<int> cluster;
			proximity<PointT>(i, cluster, cloud, pointProcessed, tree, distanceTol);
			clusters.push_back(cluster);
		}
	}
 
	return clusters;
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> Clustering<PointT>::makeClusters(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    KdTree kdtree;
    kdtree.setInputCloud<PointT>(cloud);

    std::vector<std::vector<int>> clusterIndicies = euclideanClustering<PointT>(cloud, &kdtree, clusterTolerance);

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
