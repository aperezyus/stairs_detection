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

#ifndef GLOBAL_SCENE_H
#define GLOBAL_SCENE_H

#include <boost/filesystem.hpp>
#include <pcl/PCLHeader.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/angles.h>
#include <pcl/common/common_headers.h>

#include "RGBD/plane.h"
#include "RGBD/current_scene.h"


// CLASS VISUALIZER
class GlobalScene {

  public:

    GlobalScene() {
        c2f.setIdentity();
        main_dir.setIdentity();
		
        initial_floor_ = false;
        new_floor_ = false;
        has_manhattan_ = false;

	}

    //// Resets the object
    void reset();


    //// Finds the floor in scene from given point cloud
    /// in: cloud (Pointcloud to find the floor)
    /// out: (in class) floor_normal_ (4-element vector with plane coefficients [A,B,C,D]), initial_floor_ (true if there is floor in the image)
    void findFloor(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);

    //// Performs RANSAC operation to find the floor, evaluating if first plane found is valid
    /// in: cloud (pointcloud to find the floor)
    /// out: remaining points of the cloud (not in plane found), normal (to become floor_normal_)
    /// return: true (if floor found)
	bool subtractInitialPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, Eigen::Vector4f &normal);

    //// Computes camera to floor transformation matrix given floor_normal_
    /// in: floor_normal (4-element vector with plane coefficients [A,B,C,D])
    /// out: (in class) c2f, f2c (camera to floor transformation matrix and inverse), person_height_
    void computeCamera2FloorMatrix (Eigen::Vector4f floor_normal);

    //// Find floor quickly given vPlanes
    /// in: vPlanes (vector with Planes in the scene)
    /// out: (in class) floor_normal_, new_floor (true if there is new floor in vPlanes)
    void findFloorFast(std::vector<Plane> vPlanes);

    //// Find Manhattan directions in current scene and updates w.r.t. old Manhattan directions
    /// in: scene (to compute Manhattan directions in current scene)
    /// out: (in class) updated main_dir, floor_normal_, c2f, f2c
    void getManhattanDirections(CurrentScene &scene);

    //// Given Manhattan directons from new scene, update old ones (maintaining orientaitons)
    /// in: manhattan_dirs (new Manhattan directions)
    /// out: main_dir (Principal directions), has_manhattan_ (true if Manhattan has been found)
    void updateManhattanDirections(Eigen::Matrix3f manhattan_dirs);

    //// Updates floor with new Manhattan coordinates (to be used, e.g. when there is no floor in scene)
    /// in: main_dir (Manhattan directions)
    /// out: (in class) floor_normal_, c2f, f2c
    void updateFloorWithManhattan();



	
    //// Variables
    bool initial_floor_; //  true if floor has been found in the scene
    Eigen::Vector4f floor_normal_; // plane coefficients of floor plane
    Eigen::Affine3d c2f; // transformation matrix camera to floor
    Eigen::Affine3d f2c; // transformation matrix floor to camera
    float person_height_; // estimated height of the camera to floor
    bool new_floor_; // true if floor has been found in current scene
    bool has_manhattan_; // true if manhattan directions have been found
    Eigen::Matrix3f main_dir; // Principal directions (Manhattan directions) on the global scene

	
};

#endif
