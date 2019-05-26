/**
* This file is part of stairs_detection.
*
* Copyright (C) 2019 Alejandro Pérez Yus <alperez at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/aperezyus/stairs_detection>
*
* stairs_detection is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* stairs_detection is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with stairs_detection. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef CURRENT_SCENE_H
#define CURRENT_SCENE_H

#include <boost/filesystem.hpp>
#include <pcl/PCLHeader.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common_headers.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/sample_consensus/sac_model_normal_plane.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/sample_consensus/ransac.h>

#include "RGBD/plane.h"

class CurrentScene {
public:

    CurrentScene()	{
        has_manhattan_ = false;
    }

    CurrentScene(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float leaf_size) {
        this->applyVoxelFilter(leaf_size,cloud);
        has_manhattan_ = false;
    }

    //// Applies Voxel Grid filter to cloud.
    /// in: leaf_size (size of voxel, in meters), cloud (to be filtered)
    /// out: (in class) fcloud (filtered cloud)
    void applyVoxelFilter(float leaf_size,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    //// Transforms filtered cloud to absolute coordenates
    /// in: T (transformation from current to global coordinates)
    void getAbsoluteCloud(Eigen::Affine3d T);

    //// Removes floor and ceiling parts (overhead) from absolute fcloud (normally used after getAbsoluteCloud)
    /// in: floor_height (although floor is at z = zero, a threshold is provided, e.g. 0.10m), ceiling_height
    void filterFloorAndCeiling(float floor_height, float ceiling_height);

    //// Get normals point cloud from fcloud using radius search
    /// in: radius (in metres)
    /// out: (in class)normals (pointcloud of pcl::Normal), tree (KdTree used also in region-growing)
    void getNormalsRadius(double radius);

    //// Get normals point cloud from fcloud using neighbor search
    /// in: neighbors (number of)
    /// out: normals (pointcloud of pcl::Normal), tree (KdTree used also in region-growing)
    void getNormalsNeighbors(int neighbors);

    //// Applies region growing segmentation algorithm to fcloud.
    //// For each region, applies isPlane function and determines if is a plane and creates plane object.
    /// out: (in class) vPlanes (vector with objects of class Plane), vObstacles (vector with pointclouds belonging to non-planar regions), remaining_points (points not belonging to any region given thresholds)
    void regionGrowing();

    //// Determines if a region is a plane or not, adding thus region to vPlanes or vObstacles
    /// in: region (point cloud)
    /// out: (in class) vPlanes (adds new plane) or vObstacle (if it is not a plane)
    /// return: true (if region is a plane)
    bool isPlane (pcl::PointCloud<pcl::PointXYZ>::Ptr &region);

    //// Euclidean Cluster Extraction
    /// in: points (pointcloud to extract clusters from, e.g. remaining_points)
    /// out: (in class) vObstacles (adds to the vector pointclouds belonging to non-planar regions)
    void extractClusters(pcl::PointCloud<pcl::PointXYZ>::Ptr &points);

    //// Euclidean Cluster Extraction and Planes: additionally verifies if the clusters are planar regions, and if so, adds to vPlanes
    /// in: points (pointcloud to extract clusters from, e.g. remaining_points)
    /// out: (in class) vObstacles (adds to the vector pointclouds belonging to non-planar regions), vPlanes (if pointcloud is a plane)
    void extractClustersAndPlanes(pcl::PointCloud<pcl::PointXYZ>::Ptr &points);

    //// Compute centroids for each plane in the scene (vPlanes)
    /// out: vPlanes.centroid
    void getCentroids();

    //// Compute contours for each plane in the scene (vPlanes)
    /// out: vPlanes.contour
    void getContours();

    //// Transform all coeffs from the planes in the scene (vPlanes) to floor reference
    /// in: c2f (transformation matrix from camera coordinates to floor coordinates)
    /// out: vPlanes.coeffs2f (vPlanes.coeffs transformed to floor coordinate frame)
    void getPlaneCoeffs2Floor(Eigen::Affine3d c2f);

    //// Transform all centroids from the planes in the scene (vPlanes) to floor reference
    /// in: c2f (transformation matrix from camera coordinates to floor coordinates)
    /// out: vPlanes.centroid2f (vPlanes.centroid transformed to floor coordinate frame)
    void getCentroids2Floor(Eigen::Affine3d c2f);

    //// Transform all clouds from the planes in the scene (vPlanes) to floor reference
    /// in: c2f (transformation matrix from camera coordinates to floor coordinates)
    /// out: vPlanes.cloud2f (vPlanes.cloud transformed to floor coordinate frame)
    void getClouds2Floor(Eigen::Affine3d c2f);

    //// Transform coeffs, cloud, centroid and contour from planes in the scene (vPlanes) to floor reference
    /// in: c2f (transformation matrix from camera coordinates to floor coordinates), absolute (true if initial fcloud is already absolute)
    /// out: vPlanes.centroid2f/cloud2f/coeffs2f/contour2f
    void transformPlanes(Eigen::Affine3d c2f, bool absolute);

    //// Transform obstacles in the scene (vObstacles) to floor reference (vObstacles2f)
    /// in: c2f (transformation matrix from camera coordinates to floor coordinates), absolute (true if initial fcloud is already absolute)
    /// out: vObstacles2f
    void transformObstacles(Eigen::Affine3d c2f, bool absolute);

    //// Performs transformPlanes and transformObstacles
    /// in: c2f (transformation matrix from camera coordinates to floor coordinates), absolute (true if initial fcloud is already absolute)
    /// out: vPlanes.centroid2f/cloud2f/coeffs2f/contour2f and vObstacles2f
    void transformPlanesAndObstacles(Eigen::Affine3d c2f, bool absolute);

    //// Classifies planes according to the orientation and position in floor reference (2f)
    //// Types: 0 = floor, 1 = horizontal, 2 = vertical/lateral, 3 = vertical/frontal. 5 = other/obstacle
    /// out: vPlanes.type
    void classifyPlanes();

    //// Get Manhattan directions of the scene using normals assuming known floor: X sideways, Y vertical, Z front-back
    /// in: f2c (floor to camera transformation matrix)
    /// out: main_dir (3x3 Matrix with the directions X-Y-Z)
    void getManhattanDirectionsFromNormalsWithFloor(Eigen::Affine3d f2c, Eigen::Affine3d c2f);

    //// Get Manhattan directions of the scene using normals: X sideways, Y vertical, Z front-back
    /// out: main_dir (3x3 Matrix with the directions X-Y-Z)
    void getManhattanDirectionsFromNormals();

    //// Get Manhattan directions of the scene using planes assuming known floor: X sideways, Y vertical, Z front-back
    /// in: f2c (floor to camera transformation matrix)
    /// out: main_dir (3x3 Matrix with the directions X-Y-Z)
    void getManhattanDirectionsFromPlanesWithFloor(Eigen::Affine3d f2c);

    //// Get Manhattan directions of the scene using planes: X sideways, Y vertical, Z front-back
    /// out: main_dir (3x3 Matrix with the directions X-Y-Z)
    void getManhattanDirectionsFromPlanes();

    //// Sort Manhattan directions given an intuition about how the camera is posed (slightly pointing downwards)
    /// in: best_main_dir (unordered main_dir from any getManhattanDirections function)
    /// out: main_dir (3x3 Matrix with ordered directions X-Y-Z)
    Eigen::Matrix3f sortManhattanDirections(Eigen::Matrix3f best_main_dir);


    // Variables
    pcl::PointCloud<pcl::PointXYZ>::Ptr fcloud; // Filtered point cloud
    pcl::PointCloud<pcl::Normal>::Ptr normals; // Normals point cloud
    pcl::search::Search<pcl::PointXYZ>::Ptr tree; // KdTree for fcloud
    std::vector<Plane> vPlanes; // Vector of Plane objects in the scene
    pcl::PointCloud<pcl::PointXYZ>::Ptr remaining_points; // Pointcloud of points not belonging to any plane for cluster extraction
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > vObstacles; // Vector of non-planar pointcloud clusters
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > vObstacles2f; // Vector of non-planar pointcloud clusters at floor reference
    Eigen::Matrix3f main_dir; // Manhattan directions (X-Y-Z) by columns
    bool has_manhattan_; // True if Manhattan directions could be computed
};

#endif
