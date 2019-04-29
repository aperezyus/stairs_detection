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

#include "stair/stair_classes.h"
#include "RGBD/global_scene.h"
//#include "custom_functions.h"

struct GlobalSceneStair : public GlobalScene
{
  GlobalSceneStair()
  {

    climbing = false;
    p2m.setIdentity();


  }

  bool findFloorFastClimbing(std::vector<Plane> vPlanes);
  bool findVerticalFast(std::vector<Plane> vPlanes);
  void updateFloorNormalWithOdo(Eigen::Affine3d pose);


  // Stair odometry
  bool climbing;
  Stair current_stair;
  Eigen::Affine3d f2s;
  Eigen::Affine3d initial_stair_pose;
  Eigen::Affine3d initial_p2s;
  Eigen::Affine3d initial_stair_pose_to_keep;
  Eigen::Affine3d initial_stair_manhattan;
  Eigen::Affine3d p2m;
  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr > vClouds;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr global_cloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cumulated_cloud;
  double cumulated_xt;
  double cumulated_yt;
  double cumulated_zt;
  std::vector<Eigen::Affine3d> vPoses;
  std::vector<Eigen::Vector3f> vPersonPos;
  std::vector<Eigen::Vector3f> vTranslations;
  std::vector<Eigen::Vector3f> vRotations;
  std::vector<Eigen::Vector3f> vTranslations_non_corrected;
  std::vector<Eigen::Vector3f> vRotations_non_corrected;
  std::vector<Eigen::Vector3f> vTranslations_stair;
  std::vector<Eigen::Vector3f> vRotations_stair;
  std::vector<int> vStepImOnLength;
  std::vector<int> vStepImOnHeight;
};
