/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"
#include "Clustering.h"
#include "Clustering.cpp"

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

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* processor, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
    //ProcessPointClouds<pcl::PointXYZI>* processor = new ProcessPointClouds<pcl::PointXYZI>();
    //pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = processor->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
    //renderPointCloud(viewer, inputCloud, "inputCloud");
    
    auto filterCloud = processor->FilterCloud(inputCloud, 0.25, Eigen::Vector4f(-10, -5, -2, 1), Eigen::Vector4f(30, 8, 0.5, 1));

    // calculate optimal number of itterations based on expected percentage of outliers
    int k = log(0.05) / log(1.0 - pow(0.5,3));
    std::pair<typename pcl::PointCloud<pcl::PointXYZI>::Ptr, typename pcl::PointCloud<pcl::PointXYZI>::Ptr> segres = processor->SegmentPlane(filterCloud, k, 0.25);
    //renderPointCloud(viewer, segres.first, "obs", Color(1,0,0));
    renderPointCloud(viewer, segres.second, "plane", Color(1,1,1));

    //std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = processor->Clustering(segres.first, 0.5, 10, 500);

    Clustering<pcl::PointXYZI>* clustering = new Clustering<pcl::PointXYZI>();
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = clustering->makeClusters(segres.first, 0.5, 10, 500);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};

    for(auto cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        processor->numPoints(cluster);

        renderPointCloud(viewer, cluster, "obstCloud"+std::to_string(clusterId), colors[clusterId % 3]);

        Box box = processor->BoundingBox(cluster);
        renderBox(viewer, box, clusterId);

        ++clusterId;
    }
    
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
    Lidar* lidar = new Lidar(cars, 0.0);

    pcl::PointCloud<pcl::PointXYZ>::Ptr pcloud = lidar->scan();

    //renderRays(viewer, lidar->position, pcloud);
    //renderPointCloud(viewer, pcloud, "point Cloud");
    
    // TODO:: Create point processor
    ProcessPointClouds<pcl::PointXYZ> processor;
    std::pair<typename pcl::PointCloud<pcl::PointXYZ>::Ptr, typename pcl::PointCloud<pcl::PointXYZ>::Ptr> segres = processor.SegmentPlane(pcloud, 100, 0.3);
    //renderPointCloud(viewer, segres.first, "obs", Color(1,0,0));
    renderPointCloud(viewer, segres.second, "plane", Color(1,1,1));

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = processor.Clustering(segres.first, 1.0, 3, 30);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};

    for(auto cluster : cloudClusters)
    {
        std::cout << "cluster size ";

        renderPointCloud(viewer, cluster, "obstCloud"+std::to_string(clusterId), colors[clusterId]);

        Box box = processor.BoundingBox(cluster);
        renderBox(viewer, box, clusterId);

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

    ProcessPointClouds<pcl::PointXYZI>* processor = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = processor->streamPcd("../src/sensors/data/pcd/data_1/");
    auto streamItr = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud;
    /*
    cityBlock(viewer); 
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce ();
    } 
    */

   while(!viewer->wasStopped())
   {
       viewer->removeAllPointClouds();
       viewer->removeAllShapes();

       inputCloud = processor->loadPcd((*streamItr).string());
       cityBlock(viewer, processor, inputCloud);

       streamItr++;
       if(streamItr == stream.end())
       {
           streamItr = stream.begin();
       }

       viewer->spinOnce();
   }

}