# Lidar-Obstacle-Detection
This project implements Lidar obstacle detection which filters, segments, and clusters real point cloud data to detect obstacles in a driving environment.

## Project Goal
- Process raw lidar data with filtering, segmentation, and clustering to detect other vehicles on the road
- implement Ransac with planar model fitting to segment point clouds
- implement Euclidean clustering with a KD-Tree to cluster and distinguish vehicles and obstacles

## Implementation detail 
### Lidar data

### Point cloud segmentation
- PCL to sengment point clouds
- RANSAC for planar model fitting 

### Clustering obstacles
- PCL to cluster obstacles
- KD-tree to store point cloud data 
- Euclidean Clustering with a KD-tree to find clusters and distinguish vehicles
- Building boxes around clusters


### Processing Point Cloud Data(PCD) 
- PCD (Point Cloud Data) 
- Filter PCD data
- multiple PCD files 
- apply point cloud processing to detect obtacles 
