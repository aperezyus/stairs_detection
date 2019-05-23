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

#include "stair/global_scene_stair.h"

bool GlobalSceneStair::findFloorFastClimbing(std::vector<Plane> vPlanes){
  bool floor_found = false;
  for (size_t Q=0; Q<vPlanes.size();Q++) {

    Eigen::Vector4f normal = Eigen::Vector4f(-vPlanes[Q].coeffs[0],-vPlanes[Q].coeffs[1],-vPlanes[Q].coeffs[2], -vPlanes[Q].coeffs[3]);

    float angle = acos(normal.head<3>().dot(floor_normal_.head<3>()));

    if (pcl::rad2deg(angle)<10) {
      floor_normal_ = normal;
      floor_found = true;
      return floor_found;
    }
  }
  return floor_found;
}

bool GlobalSceneStair::findVerticalFast(std::vector<Plane> vPlanes) {
  size_t n_inliers = 0;
  size_t max_n_inliers = 0;
  Eigen::Vector3f cumulated_vector(0,0,0);

  sort(vPlanes.begin(), vPlanes.end(), sortPlanesBySize);


  for (size_t P = 0; P < vPlanes.size(); P++) {

      Eigen::Vector4f normal = -Eigen::Vector4f(vPlanes[P].coeffs[0],vPlanes[P].coeffs[1],vPlanes[P].coeffs[2], vPlanes[P].coeffs[3]);

      float angle = acos(normal.head<3>().dot(floor_normal_.head<3>()));

      if (pcl::rad2deg(angle)<5) {
        n_inliers = vPlanes[P].cloud->points.size();

        cumulated_vector += Eigen::Vector3f(normal[0]*n_inliers,normal[1]*n_inliers,normal[2]*n_inliers);

        if (n_inliers > max_n_inliers) {
          max_n_inliers = n_inliers;
        }
      }
  }

  if (max_n_inliers > 0) {
    cumulated_vector.normalize();
    floor_normal_.head(3) = cumulated_vector;
    floor_normal_[3] = 1;

    return true;
  }
  else
    return false;
}

void GlobalSceneStair::updateFloorNormalWithOdo(Eigen::Affine3d pose) {

//  std::cout << "Current pose: " << std::endl << pose.matrix() << std::endl;
//  std::cout << "Previous pose: " << std::endl << initial_stair_pose.matrix() << std::endl;
//  std::cout << "Change of pose: " << std::endl << (pose.inverse()*initial_stair_pose).matrix() << std::endl;

  Eigen::Vector3d floor_normal_head;
  floor_normal_head = (pose.inverse()).rotation()*initial_stair_pose.rotation()*floor_normal_.head(3).cast<double>();
  floor_normal_.head(3) = floor_normal_head.cast<float>();
  floor_normal_.normalize();
//  std::cout << "Floor normal: " << std::endl << floor_normal_ << std::endl;
}

