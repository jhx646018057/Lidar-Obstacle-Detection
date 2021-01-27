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

    // voxel grid point reduction and region based filtering
    // Create the filtering object: downsample the dataset using a leaf size of .2m
    pcl::VoxelGrid<PointT> vg;
    typename pcl::PointCloud<PointT>::Ptr cloudFiltered (new pcl::PointCloud<PointT>);
    // std::cout << typeid(vg).name() << std::end;
   vg.setInputCloud(cloud);
   vg.setLeafSize(filterRes, filterRes, filterRes);
   vg.filter(*cloudFiltered);
   typename pcl::PointCloud<PointT>::Ptr cloudRegion (new pcl::PointCloud<PointT>);
    // Filter point cloud that is out of region of interest
   pcl::CropBox<PointT> region(true);
   region.setMin(minPoint);
   region.setMax(maxPoint);
   region.setInputCloud(cloudFiltered);
   region.filter(*cloudRegion);
   std::vector<int> indices;
   pcl::CropBox<PointT> roof(true);
   roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
   roof.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1));
   roof.setInputCloud(cloudRegion);
   roof.filter(indices);
   pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
   for(int point: indices)
   {
       inliers->indices.push_back(point);
   }
   pcl::ExtractIndices<PointT> extract;
   extract.setInputCloud (cloudRegion);
   extract.setIndices(inliers);
   extract.setNegative(true);
   extract.filter(*cloudRegion);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRegion;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // Create two new point clouds, one cloud with obstacles and other with segmented plane
  typename pcl::PointCloud<PointT>::Ptr obstCloud(new pcl::PointCloud<PointT>());
  typename pcl::PointCloud<PointT>::Ptr planeCloud(new pcl::PointCloud<PointT>());

  for (int index : inliers->indices)
    planeCloud->points.push_back(cloud->points[index]);

  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*obstCloud);
  std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);

    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // PCL implmentation to find inliers for the cloud.
  pcl::SACSegmentation<PointT> seg;
  pcl::PointIndices::Ptr inliers{new pcl::PointIndices};
  pcl::ModelCoefficients::Ptr coefficients{new pcl::ModelCoefficients};

  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(maxIterations);
  seg.setDistanceThreshold(distanceThreshold);
  seg.setInputCloud(cloud);
  seg.segment(*inliers, *coefficients);
  //end of PCL implentation
  if (inliers->indices.empty()) {
    std::cout << "could not estimate planar model" << std::endl;
  }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);

    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // PCL implementation to perform euclidean clustering to group detected obstacles
   typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
   tree->setInputCloud(cloud);
  
   std::vector<pcl::PointIndices> clusterIndices;
   pcl::EuclideanClusterExtraction<PointT> ec;
   ec.setClusterTolerance(clusterTolerance);
   ec.setMinClusterSize(minSize);
   ec.setMaxClusterSize(maxSize);
   ec.setSearchMethod(tree);
   ec.setInputCloud(cloud);
   ec.extract(clusterIndices);
     for(pcl::PointIndices getIndices: clusterIndices)
   {
      typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);
      for(int index: getIndices.indices)
       {
           cloudCluster->points.push_back ( cloud->points[index]);
       }
       cloudCluster->width = cloudCluster->points.size();
       cloudCluster->height = 1;
       cloudCluster->is_dense = true;
       clusters.push_back(cloudCluster);
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

/**** EuclideanClustering***/
template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::EuclideanClustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{
    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();
    
    ClusterPts<PointT> clusterPoints(cloud->points.size(), clusterTolerance, minSize, maxSize);
    
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters = clusterPoints.EuclidCluster(cloud);
    
    
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "KDTree eucilidean clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;
    
    return clusters;
}
//modified from segmentPlane using pkg --> own RansacPlane
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
    auto startTime = std::chrono::steady_clock::now();
    //srand(time(NULL));
    
    int num_points = cloud->points.size();
    auto all_points = cloud->points;
    
    Ransac<PointT> segRansac(maxIterations, distanceTol, num_points);
    
    // get inliers from local-RANSAC implementation rather than PCL implementation
    std::unordered_set<int> inliersResult = segRansac.Ransac3d(cloud);
    
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "RANSAC plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;
    
    typename pcl::PointCloud<PointT>::Ptr out_plane(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr in_plane(new pcl::PointCloud<PointT>());
    
    for (int i=0; i<num_points; i++) {
        PointT pt = all_points[i];
        if (inliersResult.count(i)) {
            out_plane->points.push_back(pt);
        }
        else {
            in_plane->points.push_back(pt);
        }
    }
    
    return std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>(in_plane, out_plane);
}
