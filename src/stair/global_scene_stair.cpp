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

bool GlobalSceneStair::findFloorFastClimbing(std::vector<Plane> vPlanes)
{
  bool floor_found = false;
  for (int Q=0; Q<vPlanes.size();Q++)
  {

    Eigen::Vector4f normal = Eigen::Vector4f(-vPlanes[Q].coeffs[0],-vPlanes[Q].coeffs[1],-vPlanes[Q].coeffs[2], -vPlanes[Q].coeffs[3]);

    float angle = acos(normal.head<3>().dot(floor_normal.head<3>()));
//    angle = getAngle(normal, floor_normal);

    // std::cout << floor_normal << std::endl;

    // if (pcl::rad2deg(angle)<5)
    if (pcl::rad2deg(angle)<10)
    {
      //~ std::cout << "Se ha cambiado el suelo con un angulo de " << pcl::rad2deg(angle) << std::endl << std::endl;
      floor_normal = normal;
      floor_found = true;
      current_floor = Q;
      return floor_found;
    }
  }
  return floor_found;
}

bool GlobalSceneStair::findVerticalFast(std::vector<Plane> vPlanes)
{
  float n_inliers = 0;
  int max_n_inliers = 0;
  int plane_selected = 0;
  Eigen::Vector3f cumulated_vector(0,0,0);

  sort(vPlanes.begin(), vPlanes.end(), sortPlanesBySize);


  for (int P = 0; P < vPlanes.size(); P++)
  {
    // std::cout << "Candidato " << P << std::endl;
    // if (vPlanes[P].type < 2)
    // if (vPlanes[P].coeffs2f(1)>vPlanes[P].coeffs2f(0) and fabs(vPlanes[P].coeffs2f(1))>fabs(vPlanes[P].coeffs2f(2)))
    // {
      // std::cout << "Es horizontal " << std::endl;

      Eigen::Vector4f normal = -Eigen::Vector4f(vPlanes[P].coeffs[0],vPlanes[P].coeffs[1],vPlanes[P].coeffs[2], vPlanes[P].coeffs[3]);

      float angle = acos(normal.head<3>().dot(floor_normal.head<3>()));
//      float angle;
//      angle = getAngle(normal, floor_normal);
      // std::cout << normal << std::endl;
      // std::cout << floor_normal << std::endl;

      // std::cout << "El ángulo con la normal del suelo anterior es " << pcl::rad2deg(angle) << std::endl;

      if (pcl::rad2deg(angle)<5)
      // if (pcl::rad2deg(angle)<10)
      {
        n_inliers = vPlanes[P].cloud->points.size();

        // std::cout << n_inliers << std::endl;
        // std::cout << normal << std::endl;

        cumulated_vector += Eigen::Vector3f(normal[0]*n_inliers,normal[1]*n_inliers,normal[2]*n_inliers);

        // std::cout << "Es menor que 10, hay " << n_inliers << " inliers" << std::endl;

        // for (int Q = 0; Q < vPlanes.size(); Q++)
        // {

        // 	if ((Q != P) and (vPlanes[Q].type < 2))
        // 	{
        // 		// std::cout << "Lo comparamos con el plano " << Q << ", que es también horizontal" << std::endl;
        // 		Eigen::Vector4f current_normal;
        // 		current_normal = vPlanes[Q].coeffs;
        // 		float angle;

        // 		angle = getAngle(normal,current_normal);

        // 		// std::cout << "Forman un ángulo de " << pcl::rad2deg(angle) << std::endl;

        // 		if (pcl::rad2deg(angle)<5)
        // 		{
        // 			n_inliers += vPlanes[Q].cloud->points.size();
        // 		}
        // 	}
        // }

        if (n_inliers > max_n_inliers)
        {

          // std::cout << "Se supera el número máximo de inliers, ahora hay " << n_inliers << std::endl;
          // std::cout << pcl::rad2deg(angle) << std::endl;
          max_n_inliers = n_inliers;
          // floor_normal = normal;
          // current_floor = P;
        }

        // break;
      }
    // }
  }





  if (max_n_inliers > 0)
  {
    cumulated_vector.normalize();
    floor_normal.head(3) = cumulated_vector;
    floor_normal[3] = 1;
    // std::cout << cumulated_vector << std::endl;

    return true;

  }
  else
    return false;
}

void GlobalSceneStair::updateFloorNormalWithOdo(Eigen::Affine3d pose)
{

  std::cout << "Pose actual: " << std::endl << pose.matrix() << std::endl;
  std::cout << "Pose anterior: " << std::endl << initial_stair_pose.matrix() << std::endl;
  std::cout << "Cambio de pose: " << std::endl << (pose.inverse()*initial_stair_pose).matrix() << std::endl;

  Eigen::Vector3d floor_normal_head;
  floor_normal_head = (pose.inverse()).rotation()*initial_stair_pose.rotation()*floor_normal.head(3).cast<double>();
  floor_normal.head(3) = floor_normal_head.cast<float>();
  floor_normal.normalize();
  std::cout << "Floor normal: " << std::endl << floor_normal << std::endl;
}

