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
//#include "stair/stair_classes.h"


// CLASS VISUALIZER
struct GlobalScene
{
	// Constructor with standard settings
	GlobalScene()
	{
    c2f.setIdentity();
    f2f.setIdentity();
    c02a.setIdentity();
    eigDx.setIdentity();
		
        initial_floor = false;
        initial_reference = false;
        new_floor = false;
        has_manhattan = false;

	}

  //// Finds the floor in scene from given point cloud
  /// in: cloud (Pointcloud to find the floor)
  /// out: floor_normal, initial_floor (true if there is floor in the image);
	void findFloor(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);

  //// Performs RANSAC operation to find the floor
  /// in: cloud (pointcloud to find the floor)
  /// out: normal (to become floor_normal), true (if floor found)
	bool subtractInitialPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, Eigen::Vector4f &normal);

  //// Computes camera to floor transformation matrix given floor_normal
  /// in: floor_normal
  /// out: c2f, f2c (camera to floor transformation matrix and inverse), person_height
  void computeCamera2FloorMatrix ();

  //// Computes camera to absolute camera reference (i.e. map reference, from gazebo, with Z upwards)
  /// in: f2a (transformation matrix from floor reference frame (Y upwards) to absolute reference frame (Z upwards)
  /// out: c2a, a2c (camera to absolute transformation matrix and inverse)
  void computeCamera2AbsoluteMatrix (Eigen::Affine3d f2a);

  //// Update camera to absolute matrix with odometry values
  /// in: c2c0 (odometry, i.e. transformation matrix from current camera (c) to initial camera (c0))
  /// out: updated c2a, a2c
  void computeCamera2AbsoluteMatrixWithOdometry (Eigen::Affine3d c2c0);

  //// Update c2f given new floor_normal and compute variation w.r.t. old c2f
  /// in: floor_normal
  /// out: updated c2f, f2c, and f2f (transformation between old and current floor)
  void computeIncrementalCamera2FloorMatrix ();

  //// Find floor quickly given vPlanes
  /// in: vPlanes (vector with Planes in the scene)
  /// out: floor_normal, new_floor (true if there is new floor in vPlanes), current_floor (index of vPlanes)
  void findFloorFast(std::vector<Plane> vPlanes);

  //// Given Manhattan directons from new scene, update old ones (maintaining orientaitons)
  /// in: manhattan_dirs (new Manhattan directions)
  /// out: eigDx (Principal directions), has_manhattan (true if Manhattan has been found)
  void updateManhattanDirections(Eigen::Matrix3f manhattan_dirs);

  //// Updates floor with new Manhattan coordinates (to be used, e.g. when there is no floor in scene)
  /// in: eigDx (Manhattan directions)
  /// out: floor_normal, c2f, f2c
  void updateFloorWithManhattan();

  //// Find Manhattan directions in current scene and updates w.r.t. old Manhattan directions
  /// in: scene (to compute Manhattan directions in current scene)
  /// out: updated eigDx, floor_normal, c2f, f2c
  void getManhattanDirections(CurrentScene &scene);

	
  //// Variables
  bool initial_floor; //  true if floor has been found in the scene
  Eigen::Vector4f floor_normal; // plane coefficients of floor plane
  Eigen::Affine3d c2f; // transformation matrix camera to floor
  Eigen::Affine3d f2c; // transformation matrix floor to camera
  float person_height; // estimated height of the camera to floor
  Eigen::Affine3d c2a; // camera to floor/manhattan in absolute coordinates (Z upwards)
  Eigen::Affine3d a2c; // floor/manhattan in absolute coordinates (Z upwards) to camera
  Eigen::Affine3d f2f; // transformation matrix amongst new deteciton of floor
  bool new_floor; // true if floor has been found in current scene
  int current_floor; // index in vPlanes of current floor
  bool has_manhattan; // true if manhattan directions have been found
  Eigen::Matrix3f eigDx; // Principal directions (Manhattan directions) on the global scene
  bool initial_reference; // true if floor/principal directions have been found the first time, establishin camera c0
  Eigen::Affine3d c02a; // Transformation from initial camera (c0) to absolute coordinates
	
	public:
	  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif
