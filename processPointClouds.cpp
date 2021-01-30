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

    // Fill in the function to do voxel grid point reduction and region based filtering
  // Create the filtering object: downsample the dataset using a leaf size of .2m
  //Voxel Grid filtering 
   pcl::VoxelGrid<PointT> vg;
   typename pcl::PointCloud<PointT>::Ptr cloudFiltered (new pcl::PointCloud<PointT>);
   // std::cout << typeid(vg).name() << std::end;
   vg.setInputCloud(cloud);
   vg.setLeafSize(filterRes, filterRes, filterRes);
   vg.filter(*cloudFiltered);
  
  //region Filtering 
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
//extra root point filtering
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
   //std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloud, cloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	//pcl::PointIndices::Ptr inliers;
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
    //std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(cloud,cloud);
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

/****Ransance3D segmentation***/
//modified from segmentPlane using pkg --> own RansacPlane
// identify the cluster of point with argument: min, max points in the cloud and meet the cluster tolerance requirement.
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
    auto startTime = std::chrono::steady_clock::now();
    srand(time(NULL));
    
    int num_points = cloud->points.size();
    auto all_points = cloud->points;
    
    //TODO: OOP refactoring later --> Ransac algorithm
    //Ransac<PointT> segRansac(maxIterations, distanceTol, num_points);
    // get inliers from local-RANSAC implementation rather than PCL implementation
    //std::unordered_set<int> inliersResult = segRansac.Ransac3d(cloud);
    std::unordered_set<int> inliersResult;
    /*** For max iterations
    // Randomly sample subset and fit line
    // Measure distance between every point and fitted line
    // If distance is smaller than threshold count it as inlier
    // Return indicies of inliers from fitted line with most inliers ***/
    while (maxIterations--)  //
    {
        std::unordered_set<int> inliers;
        while (inliers.size() < 3)
        {
            inliers.insert(rand() % (cloud->points.size()));
        }
        
        // extract 3-points
        float x1, y1, z1, x2, y2, z2,x3, y3, z3;
        auto itr = inliers.begin();  //auto automatic type
        x1 = cloud->points[*itr].x;
        y1 = cloud->points[*itr].y;
        z1 = cloud->points[*itr].z;
        
        itr++;
        x2 = cloud->points[*itr].x;
        y2 = cloud->points[*itr].y;
        z2 = cloud->points[*itr].z;
        
        itr++;
        x3 = cloud->points[*itr].x;
        y3 = cloud->points[*itr].y;
        z3 = cloud->points[*itr].z;
        
        // plane equation coefficients
        float a, b, c, d, v3_i, v3_j, v3_k ;
        v3_i = (y2 - y1)*(z3 - z1) - (z2 - z1)*(y3 - y1);
        v3_j = (z2 - z1)*(x3 - x1) - (x2 - x1)*(z3 - z1);
        v3_k = (x2 - x1)*(y3 - y1) - (y2 - y1)*(x3 - x1);
        a = v3_i;
        b = v3_j;
        c = v3_k;
        d = -(v3_i*x1 + v3_j*y1 + v3_k*z1);
        
        // RANSAC point-to-plane distance check
        for (int index = 0; index < cloud->points.size(); index++)
        {
            if (inliers.count(index) > 0)
                continue;
            
            //pcl::PointXYZ point = cloud->points[index]; error not-machine with kdtree3d.search
            PointT point = cloud->points[index] ;
            float x4 = point.x;
            float y4 = point.y;
            float z4 = point.z;
            float distance = fabs(a*x4 + b*y4+ c*z4+ d) / sqrt(a*a+b*b+c*c);
            
            if (distance <= distanceTol)
            {
                inliers.insert(index);
            }
        }
        //inliersResult is the point cloud on the ground
        if (inliers.size() > inliersResult.size())
        {
            inliersResult = inliers;
        }
    }
    
    if (inliersResult.size () == 0)
    {
        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }
 
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
    
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "RANSAC plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    
    return std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>(in_plane, out_plane);
}


/****KdTree3D EuclideanClustering***/
template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::EuclideanClustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{
    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    
    // Create the KdTree3D object using the points in cloud.
    typename KdTree3D<PointT>::KdTree3D *tree =new KdTree3D<PointT>;
    tree->insert(cloud);
  
    // PCL implementation to perform euclidean clustering to group detected obstacles
    std::vector<std::vector<int>> clusterIndices = euclideanClusterHelper(cloud, tree, clusterTolerance , minSize, maxSize);

    for (std::vector<std::vector<int>>::const_iterator it = clusterIndices.begin (); it != clusterIndices.end (); ++it)
    {
        typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);
        for (std::vector<int>::const_iterator index = it->begin (); index != it->end (); ++index) // for(int index: getIndices.indices)
        {
            cloudCluster->points.push_back (cloud->points[*index]);
        }
        cloudCluster->width = cloudCluster->points.size ();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;
        clusters.push_back(cloudCluster);
    }
    
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "KDTree3D eucilidean clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;
    
    return clusters;
}


// ADDED PROXIMITY: Identify all the points within distanceTol from the given point and return the indices of the points: recursive
template<typename PointT>
void ProcessPointClouds<PointT>::Proximity(typename pcl::PointCloud<PointT>::Ptr cloud,std::vector<int> &cluster, std::vector<bool> &processed_f,int idx,typename KdTree3D<PointT>::KdTree3D* tree,float distanceTol, int maxSize)
{
    if((processed_f[idx]==false)&&(cluster.size()<maxSize))
    {
        processed_f[idx]=true;
        cluster.push_back(idx);
        std::vector<int> nearby = tree->search(cloud->points[idx],distanceTol);
        for(int index : nearby)
        {
            if(processed_f[index]==false)
            {
                Proximity(cloud, cluster,processed_f,index,tree,distanceTol,maxSize);
            }
        }
    }
    
}

// Added euclideanCluster: identify clusters that have points with in min and max limits
template<typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanClusterHelper(typename pcl::PointCloud<PointT>::Ptr cloud, typename KdTree3D<PointT>::KdTree3D* tree, float distanceTol, int minSize, int maxSize)
{
    std::vector<std::vector<int>> clusters;
    //Create a flag for each point in the cloud, to identified if the point is processed or not, and set it to false
    std::vector<bool> processed_flag(cloud->points.size(),false);
    
    //Loop through each point of the cloud
    for(int idx=0;idx<cloud->points.size();idx++)
    {
        //Pass the point to Proximity function only if it was not processed either added to a cluster or discarded)
        if(processed_flag[idx]==false)
        {
            std::vector<int> cluster;
            // Call Proximity function to identify all the points that are within distanceTol distance
            Proximity(cloud, cluster,processed_flag,idx,tree,distanceTol,maxSize);
            // Check if the number of points in the identified cluster are with in min and max Size
            if((cluster.size()>=minSize)&&cluster.size()<=maxSize)
                clusters.push_back(cluster);
            /*else
             std::cerr<<"discarted cluster"<<cluster.size()<<std::endl;*/
        }
        
    }
    /*std::cout<<"Distance Tolerance"<<distanceTol<<std::endl;
     std::cout<<"Max Distance "<<tree->max_distance<<std::endl;*/
    return clusters;
}