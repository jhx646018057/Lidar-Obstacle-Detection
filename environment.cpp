
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors
#include "processPointClouds.h"
#include "processPointClouds.cpp"
#include "sensors/lidar.h"
#include "render/render.h"

// using templates for processPointClouds so also include .cpp to help linker
// TBD : Clean up code 

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
//void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer)
void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------
    
    //ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    //pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
    //renderPointCloud(viewer,inputCloud,"inputCloud");
    
    //constexpr float X{ 30.0 }, Y{ 6.5 }, Z{ 2.5 };
    //const pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud{ pointProcessor->FilterCloud(
    //inputCloud, 0.1f, Eigen::Vector4f(-(X / 2), -Y, -Z, 1), Eigen::Vector4f(X, Y, Z, 1)) };
    //auto filterCloud = pointProcessorI->FilterCloud( inputCloud, 0.3f, Eigen::Vector4f(-(X / 2), -Y, -Z, 1), Eigen::Vector4f(X, Y, Z, 1));
    
    //filter cloud hyperparameters
    float filterRes = 0.3;
    Eigen::Vector4f minPoint(-10, -5, -2,1);
    Eigen::Vector4f maxPoint(30, 8, 1, 1);
    //ransac segment plane hyperparameters
    int maxIter = 25; //40
    float distanceThreshold = 0.3;
    // cluster hyperparameters
    float clusterTolerance = 0.53;
    int minClusterSize = 10;
    int maxClusterSize = 500; //140
    
    //step1. FilterCloud
    auto filterCloud = pointProcessorI->FilterCloud( inputCloud, filterRes, minPoint, maxPoint);
    
    //std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud=pointProcessor.SegmentPlane(filterCloud,100,0.2);
    
    
    //step2. RansacPlane (Ransac Segment)
    
    //std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->SegmentPlane(filterCloud,maxIter,distanceThreshold);//filterCloud,20,0.2--> inputCloud,25, 0.3
    //auto segmentCloud = pointProcessorI->SegmentPlane(filterCloud,maxIter,distanceThreshold);
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud =
    pointProcessorI->RansacPlane(filterCloud,maxIter,distanceThreshold);
    if (segmentCloud.first->empty() || segmentCloud.second->empty()) { return; }
    //renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1,0,0));
    //renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0,1,0));
    //renderPointCloud(viewer,filterCloud,"filterCloud", Color(0,1,0));
    renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));
    
    //Step3. Cluster the obstacle cloud  --> EuclideanClustering
    
    const std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->EuclideanClustering(segmentCloud.first, clusterTolerance, minClusterSize, maxClusterSize);
    //const std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(segmentCloud.first, clusterTolerance, minClusterSize, maxClusterSize); // //0.3,30,1775--> 0.53, 10,500
    //auto cloudClusters = pointProcessorI->Clustering(segmentCloud.first, clusterTolerance, minClusterSize, maxClusterSize);
    if (cloudClusters.empty()) { return; }
    
    int clusterId=0 ;
    
    const std::vector<Color> colors = { Color(1, 0, 1), Color(0, 1, 1), Color(1, 1, 0) };
    
    Box host_box = {-1.5, -1.7, -1, 2.6, 1.7, -0.4};
    renderBox(viewer, host_box, 0, Color(0.5, 0, 1), 0.8);
    
    //Step4.  Bouding Box for the clusters
    for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters) {
        std::cout << "cluster size" ;
        pointProcessorI->numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors.at(clusterId%colors.size()));
        
        Box box(pointProcessorI->BoundingBox(cluster));
        // [extra] Filter out some cluster with little points and shorter in height
        // [extra] if (box.z_max - box.z_min >= 0.75 || cluster->points.size() >= minClusterSize * 2) {
        
        renderBox(viewer, box, clusterId);
        //[extra] }
        
        ++clusterId;
        
    }
    //renderPointCloud(viewer,filterCloud,"filterCloud");
    
}

void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false; //ture-->showing cars in blue
    /***clustering parameter***/
    bool render_obst = false;
    bool render_plane = false;
    bool render_clusters = true;
    bool render_box = true;
    /***clustering parameter***/
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // Create lidar sensor
    Lidar* lidar = new Lidar(cars, 0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = lidar->scan();
    //renderRays(viewer, lidar->position, inputCloud);
    renderPointCloud(viewer, inputCloud, "inputCloud");
    
    // Create point processor
    ProcessPointClouds<pcl::PointXYZ> pointProcessor;
    //segment cloud for road plane isolation
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor.SegmentPlane(inputCloud, 100, 0.2);
    if(render_obst)
        renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(1,0,0));
    if(render_plane)
        renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));
    ///clustering
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor.Clustering(segmentCloud.first, 1.0, 3, 30);
    
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};
    
    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        if(render_clusters)
        {
            std::cout << "cluster size ";
            pointProcessor.numPoints(cluster);
            renderPointCloud(viewer, cluster, "obstCloud"+std::to_string(clusterId), colors[clusterId%colors.size()]);
        }
        if(render_box)
        {
            Box box = pointProcessor.BoundingBox(cluster);
            renderBox(viewer, box, clusterId);
        }
        ++clusterId;
    }
    renderPointCloud(viewer,segmentCloud.second,"planeCloud");
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
    //CameraAngle setAngle = XY;
    CameraAngle setAngle = FPS;
    initCamera(setAngle, viewer);
    //Create Point Cloud Processor
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI ;
    //ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_2");
    
    //src/sensors/data/pcd/data_1
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;
    
    while (!viewer->wasStopped ())
    {
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();
        
        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessorI, inputCloudI);
        
        streamIterator++;
        if(streamIterator == stream.end())
            streamIterator = stream.begin();
        
        viewer->spinOnce ();
    }
    /****
     //simpleHighway(viewer);
     cityBlock(viewer);
     while (!viewer->wasStopped ())
     {
     viewer->spinOnce ();
     }
     ***/
}
