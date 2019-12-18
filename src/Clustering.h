#ifndef CLUSTERING_H_
#define CLUSTERING_H_

#include <iostream> 
#include <string>  
#include <vector>
#include <ctime>
#include <chrono>
#include <pcl/common/common.h>
#include "KdTree.h"

template<typename PointT>
class Clustering {
    public:

    Clustering();

    ~Clustering();
    
    std::vector<typename pcl::PointCloud<PointT>::Ptr> makeClusters(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize);
};

#endif /* CLUSTERING_H_ */