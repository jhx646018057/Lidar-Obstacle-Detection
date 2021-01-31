# Lidar-Obstacle-Detection
This project implements Lidar obstacle detection which filters, segments, and clusters real point cloud data to detect obstacles in a driving environment. Implemented segmentation using the 3D RANSAC algorithm from scratch and Eucleadian Clustering along with the KD-Tree algorithm from scratch rather than using PCD libraries to to detect car and trucks on a narrow street using Lidar. The detection pipeline follows  filtering, segmentation, clustering, and bounding boxes.

## Project Goal
The goal of this project is to detect car and trucks on a narrow street using lidar data and points clouds.
The road is segmented from the vehicles and there are boxes placed around the detected obstacles on the road.
* stream back multiple pcd files and perform filtering, segmentation, clustering, and bounding box detection
* make the pipeline even more robust by tracking detections over the history of frames
* create associations between detections  in two different frames and use that to track objects.

## Background  : Lidar data and sensor fusion
**Lidar** sensing gives high resolution data by sending out thousands of laser signals. These lasers bounce off objects, returning to the sensor where we can determine how far away objects are how long it takes for the signal to return. Also we can tell an object that was hit by measuring the intesity of the returned signal. Each laser ray is in the infrared spectrum, and is sent out at many different angles, usually in a 360 degree range. While lidar sensors gives very high accurate models in 3D, they are currently very expensive sensors.

**Radar** data is typically very sparse and in a limited range, however it can directly tell us how fast an object is moving in a certain direction. This ability makes radars a very pratical sensor for doing things like cruise control where its important to know how fast the front car is traveling. Radar sensors are also very affordable and common in newer cars.

**Sensor Fusion** by combing lidar's high resoultion imaging with radar's ability to measure velocity of objects we can get a better understanding of the sorrounding environment than we could using one of the sensors alone.

## Implementation Approach
 The detection pipeline is composed of filtering, segmentation, clustering, and bounding boxes
### Code Structure
```
src
|-render
       |-box.h - this file has the struct definitions for box objects
       |-render.h/render.cpp - define the classes and methods for rendering objects onto the screen
|-sensors
       |-lidar.h - has functions simulating lidar sensing and creating point cloud data using ray casting
       |-data - this directory contains pcd data
|-cluster
       |-cluster.cpp 
|-kdtree3d.h 
|-environment.cpp - the main file for creating pcl viewer::a processPointClouds object and processing and visualizing pcd
|-processPointClouds.h/processPointClouds.cpp - Functions for filtering, segmenting, clustering, boxing, loading, to process the pcd and saving pcd.
```
### Load 3D PCD(point cloud data) stream 
the point cloud input will vary from frame to frame, so input point cloud will now become an input argument for the processor
streamPcd a folder directory that contains all the sequentially ordered pcd files , and it returns a chronologically ordered vector of all those file names, called stream. 
 pcd files are located in src/sensors/data/
```code
  ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
  //pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;  = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd"); //one singke pcd file
  std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1"); //pcd stream
  auto streamIterator = stream.begin();
  renderPointCloud(viewer,inputCloud,"inputCloud");
```
1. create a point processor for intensity point clouds `pcl::PointXYZI`
2. render the real pcd in `environment.cpp` using `renderPointCound()`
### Step1. Filtering with PCL
 One way to create associations between two different frames is by how close in proximity two detections are to each other and how similar they look. There are also other filtering procedures such as looking at detection that are seen in consecutive frames before they are considered.  Another way is to filter based on bounding boxes, their volume and shapes. In this project, detail bounding boxes filtering is applied as follows: 
 1. Voxel grid filtering will create a cubic grid and will filter the cloud by only leaving a single point per voxel cube, so the larger the cube length the lower the resolution of the point cloud.
 ```code
   pcl::VoxelGrid<PointT> vg;
   vg.setInputCloud(cloud);
   vg.filter(*cloudFiltered);
```
 2. Region of interest: A boxed region is defined and any points outside that box are removed.
 ```
    pcl::CropBox<PointT> region(true);
   region.setInputCloud(cloudFiltered);
   region.filter(*cloudRegion);
 ```
 3. Call filter function in `pointProcessorI` in the `environment.cpp`
> the point process function FilterCloud: The arguments to this function is the input cloud, voxel grid size, and min/max points representing the region of interest. The function returns the downsampled cloud with only points that were inside the region specified.
```code
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
...
filterCloud = pointProcessorI->FilterCloud(inputCloud, ? , Eigen::Vector4f (?, ?, ?, 1), Eigen::Vector4f ( ?, ?, ?, 1));
renderPointCloud(viewer,filterCloud,"filterCloud");
```
 
### Step2. Point cloud segmentation(RANSAC) 
- segment point clouds:  the filtered cloud is segmented into two parts, road(in green) and obstacles (in red, colors),  with points only in the filtered region of interest.
- RANSAC(random sample consensus)  for planar model fitting: RANSAC stands for Random Sample Consensus, and is a method for detecting outliers in data. RANSAC runs for a max number of iterations, and returns the model with the best fit. Each iteration randomly picks a subsample of the data and fits a model through it, such as a line or a plane. Then the iteration with the highest number of inliers or the lowest noise is used as the best model.
> `SegmentPlane` function in src/processPointClouds.cpp
```code
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);
// function to segment cloud into two parts, the drivable plane and obstacles 
std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor->SegmentPlane(inputCloud, 100, 0.2);
renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(1,0,0));
renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));
...
```

### Step3. Clustering the obstacle clouds
Next step is to cluster the obstacle cloud based on the proximity of neighboring points.  The challenges with clustering based on proximity is a big object  can be recognized in separate clusters. For example, a big truck can be broken up into two, front and back. It would be resolved by increasing the distance tolerance however, it would cause another problem such as truck and parked car would be grouped together.
- PCL to cluster obstacles
- KD-tree to store point cloud data : To do a nearest neighbor search efficiently, KD-tree data sturcture is applied(O(log(n)) since it is tree-search. By grouping points into regions in a KD-Tree, calculating distance for possibly thousands of points can be avoided.A KD-Tree is a binary tree that splits points between alternating axes. By separating space by splitting regions, nearest neighbor search can be made much faster when using an algorithm like euclidean clustering.
 > src/quiz/cluster/kdtree
`cluster.cpp` there is a function for rendering the tree after points have been inserted into it.
- Euclidean Clustering with a KD-tree to find clusters and distinguish vehicles
> euclideanCluster function returns a vector of vector ints, this is the list of cluster indices.
```code
std::vector<std::vector<int>> clusters = euclideanCluster(points, tree, 3.0);
```

### Step4. Find Bounding Boxes for the clusters
Last step is to place bounding boxes around the individual clusters. Bounding boxes enclose vehicles, and the pole on the right side of the vehicle,  one box per detected object. The function BoundingBox looks at the min and max point values of the input cloud and stores those parameters in a box struct container. To render bounding boxes around the clusters below codes are inside the loop that renders clusters in `environment.cpp`.
- Building boxes around clusters
```code 
Box box = pointProcessor->BoundingBox(cluster);
renderBox(viewer,box,clusterId);
```
## Result
Data1(src/sensors/data/pcd/data_1) and Data2 result ( src/sensors/data/pcd/data_2) 
* Data1 is to detect/track vehicles, and the pole on the right side of the vehicle enclosed by Bounding Boxes. There is one box per detected object.
* Data2 is to detect/track a bicyclist riding in front of the car, along with detecting/tracking the other surrounding obstacles in the scene.

<img src="0130data1FPS.gif" alt="Data1" width="400"/>      <img src="0130data2FPS.gif" alt="Data2" width="400"/>
 
## Challenge and next shortcomings
what if the cluster was a very long rectangular object at a 45 degree angle to the X axis. The resulting bounding box would be a unnecessarily large, and would constrain the car's available space to move around. PCA, principal component analysis and including Z axis rotations would be helpful. A challenge problem(src/sensors/data/pcd/data_2 to  detect/track a bicyclist riding in front of the car, along with detecting/tracking the other surrounding obstacles in the scene.) is then to find the smallest fitting box but which is oriented flat with the XY plane.

## Runtime environment 
### Dependency
* [PCL v1.2](https://github.com/PointCloudLibrary/pcl)
The Point Cloud Library (PCL) is a standalone, large scale, open source project for 2D/3D image and point cloud processing. PCL is widely used in the robotics community for working with point cloud data. There are a lot of built in functions in PCL that can help to detect obstacles such as Segmentation, Extraction, and Clustering.
* [Point Cloud tutorial](https://pcl.readthedocs.io/projects/tutorials/en/latest/using_pcl_pcl_config.html#using-pcl-pcl-config)
