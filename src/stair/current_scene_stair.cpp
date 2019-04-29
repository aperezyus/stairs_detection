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

#include "stair/current_scene_stair.h"

bool sortByHeight(const Plane &lstep, const Plane &rstep)
{
  return fabs(lstep.centroid2f.y) < fabs(rstep.centroid2f.y); // <
}


bool CurrentSceneStair::checkForStairs()
{
  bool are_stairs = false;
  //upstair = new Stair();
  //downstair = new Stair();

  for (int Q = 0; Q<vPlanes.size(); Q++)
  {
    if (vPlanes[Q].type <= 1)
    {
      //std::cout << vPlanes[Q].centroid.y << std::endl;
      // std::cout << vPlanes[Q].centroid2f.y << std::endl;
      if (vPlanes[Q].centroid2f.y > 0.10f)
      // if (vPlanes[Q].centroid2f.y > 0.09f)
      {
        upstair.vPlanes.push_back(vPlanes[Q]);
        // if (vPlanes[Q].centroid2f.y < 0.32f)
        if (vPlanes[Q].centroid2f.y < 0.225f)
          are_stairs = true;
      }
      // else if (vPlanes[Q].centroid2f.y < -0.09f)
      else if (vPlanes[Q].centroid2f.y < -0.10f)
      {
        downstair.vPlanes.push_back(vPlanes[Q]);
        // if (vPlanes[Q].centroid2f.y > -0.32f)
        if (vPlanes[Q].centroid2f.y > -0.225)
          are_stairs = true;
      }
      else if ((vPlanes[Q].centroid2f.y > -0.07f) and (vPlanes[Q].centroid2f.y < 0.07f))
      {
        upstair.vPlanes.push_back(vPlanes[Q]);
        downstair.vPlanes.push_back(vPlanes[Q]);
      }
    }
  }
  return are_stairs;
}

bool CurrentSceneStair::checkForStairsAbs()
{
        bool are_stairs = false;
        //upstair = new Stair();
        //downstair = new Stair();

        for (int Q = 0; Q<vPlanes.size(); Q++)
        {
                if (vPlanes[Q].type <= 1)
                {
                        //std::cout << vPlanes[Q].centroid.y << std::endl;
                        // std::cout << vPlanes[Q].centroid2f.y << std::endl;
                        if (vPlanes[Q].centroid2f.z > 0.10f)
                        // if (vPlanes[Q].centroid2f.y > 0.09f)
                        {
                                upstair.vPlanes.push_back(vPlanes[Q]);
                                // if (vPlanes[Q].centroid2f.y < 0.32f)
                                if (vPlanes[Q].centroid2f.z < 0.225f)
                                        are_stairs = true;
                        }
                        // else if (vPlanes[Q].centroid2f.y < -0.09f)
                        else if (vPlanes[Q].centroid2f.z < -0.10f)
                        {
                                downstair.vPlanes.push_back(vPlanes[Q]);
                                // if (vPlanes[Q].centroid2f.y > -0.32f)
                                if (vPlanes[Q].centroid2f.z > -0.225)
                                        are_stairs = true;
                        }
                        else if ((vPlanes[Q].centroid2f.z > -0.07f) and (vPlanes[Q].centroid2f.z < 0.07f))
                        {
                                upstair.vPlanes.push_back(vPlanes[Q]);
                                downstair.vPlanes.push_back(vPlanes[Q]);
                        }
                }
        }
        return are_stairs;
}

bool CurrentSceneStair::getLevelsFromCandidates(Stair & stair, Eigen::Affine3d c2f)
{
  bool verbose = false;

  if (stair.vPlanes.size() <= 1)
    return false;
  bool are_stairs = true;
  if (verbose)
  {
    std::cout << "hay actualmente " << stair.vPlanes.size() << " escalones" << std::endl;
    std::cout << "el primero está a " << stair.vPlanes[0].centroid2f.y << std::endl;
  }

  sort(stair.vPlanes.begin(), stair.vPlanes.end(), sortByHeight); // Se ordenan los planos descendientes por la altura con respecto al suelo, más próximos al suelo primero.
  if (verbose)
  std::cout << "tras ordenar, el primero está a " << stair.vPlanes[0].centroid2f.y << std::endl;

  int max_level = 0;

  if (stair.vPlanes.size() > 1) // Si hay más de un posible peldaño
  {
    for (int Q=0; Q<(stair.vPlanes.size()); Q++) // Se recorren todos ellos
    {
      if (verbose)
        std::cout << "NUevo escalón, a una altura de " << fabs(stair.vPlanes[Q].centroid2f.y) << std::endl;
      if (fabs(stair.vPlanes[Q].centroid2f.y) < 0.10f)
      {
        if (verbose)
          std::cout << " Parece que es suelo, bien" << std::endl;
        // Es suelo, constituye nivel 0
        if (stair.vLevels.size() == 0)
        {
          if (verbose)
            std::cout << "creo nuevo suelo" << std::endl;
          Plane temp_plane;
          temp_plane.cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
          *temp_plane.cloud = *stair.vPlanes[Q].cloud;
          temp_plane.centroid = stair.vPlanes[Q].centroid;
          temp_plane.centroid2f = stair.vPlanes[Q].centroid2f;
          temp_plane.coeffs = stair.vPlanes[Q].coeffs;
          stair.vLevels.push_back(temp_plane);

          //max_level++;
        }
        else
        {
          if (verbose)
            std::cout << "como ya tenía suelo, lo agrego" << std::endl;
          *stair.vLevels[0].cloud += *stair.vPlanes[Q].cloud;
        }
      }
      else if (stair.vLevels.size() == 0) // caso si no hay suelo, primer escalón
      {
        // Creación de suelo genérico
        Plane temp_plane;
        temp_plane.cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
        stair.vLevels.push_back(temp_plane);

        //max_level++;
        if (verbose)
          std::cout << "vale, no hay suelo y este es el primer escalón, qué nervios" << std::endl;
        if (fabs(stair.vPlanes[Q].centroid2f.y) < 0.25f)
        {
          if (verbose)
            std::cout << "cumple requisitos, NUEVO NIVEL" << std::endl;
          //Plane temp_plane;
          temp_plane.cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
          *temp_plane.cloud = *stair.vPlanes[Q].cloud;
          temp_plane.centroid = stair.vPlanes[Q].centroid;
          temp_plane.centroid2f = stair.vPlanes[Q].centroid2f;
          temp_plane.coeffs = stair.vPlanes[Q].coeffs;
          stair.vLevels.push_back(temp_plane);

          max_level++;
        }
        else
        {
          if (verbose)
            std::cout << "vale, no hay ninguno que satisfaga, no hay escalera" << std::endl;
          are_stairs = false;
          return are_stairs;
        }
      }
      else if (stair.vLevels.size() == 1) // caso de escalones que no sean suelo cuando hay suelo
      {
        if (verbose)
          std::cout << "Hay suelo, este es el primero no suelo" << std::endl;
        if (neighbour_search(stair.vLevels[0].cloud,stair.vPlanes[Q].centroid,0.50f) > 10)
        {
          if (verbose)
            std::cout << "está cerca del suelo! NUEVO NIVEL" << std::endl;
          Plane temp_plane;
          temp_plane.cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
          *temp_plane.cloud = *stair.vPlanes[Q].cloud;
          temp_plane.centroid = stair.vPlanes[Q].centroid;
          temp_plane.centroid2f = stair.vPlanes[Q].centroid2f;
          temp_plane.coeffs = stair.vPlanes[Q].coeffs;
          stair.vLevels.push_back(temp_plane);

          max_level++;
        }
        else if (fabs(stair.vPlanes[Q].centroid2f.y) > 0.25f)
        {
          if (verbose)
            std::cout << "vaya, este tampoco está en contacto y ya es lo bastante alto, no hay escalera" << std::endl;
          are_stairs = false;

          return are_stairs;
        }
      }
      else if (stair.vLevels.size() > 1) // caso genérico, hay suelo u otro escalón antes
      {
        if (verbose)
          std::cout << "Hay al menos un nivel, a ver qué pasa con este" << std::endl;
        bool is_near = false;
        for (int P = max_level; P >= 0; P--)
        {
          if ((P == 0) and (stair.vLevels[P].cloud->points.size() == 0))
            break;

          is_near = (neighbour_search(stair.vLevels[P].cloud,stair.vPlanes[Q].centroid,0.50f) > 5);
          // is_near = true;

          if (is_near)
            break;
        }

        if (is_near)
        {
          if (verbose)
            std::cout << "está cerca de alguno anterior!" << std::endl;
          if (fabs(fabs(stair.vPlanes[Q].centroid2f.y)-fabs(stair.vLevels[max_level].centroid2f.y)) < 0.07) // Si la diferencia entre niveles es mínima, se trata del mismo nivel
          {
            if (verbose)
              std::cout << "..y al mismo nivel, se agrega" << std::endl;
            Eigen::Vector3f new_normal = stair.vLevels[max_level].coeffs.head(3)*stair.vLevels[max_level].cloud->points.size() + stair.vPlanes[Q].coeffs.head(3)*stair.vPlanes[Q].cloud->points.size();
            new_normal.normalize();
            stair.vLevels[max_level].coeffs.head(3) = new_normal;
            *stair.vLevels[max_level].cloud += *stair.vPlanes[Q].cloud;
            stair.vLevels[max_level].getCentroid();
            stair.vLevels[max_level].getCentroid2Floor(c2f);


          }
          else
          {
            if (verbose)
              std::cout << "wow, estamos ante un NUEVO NIVEL" << std::endl;
            Plane temp_plane;
            temp_plane.cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
            *temp_plane.cloud = *stair.vPlanes[Q].cloud;
            temp_plane.centroid = stair.vPlanes[Q].centroid;
            temp_plane.centroid2f = stair.vPlanes[Q].centroid2f;
            temp_plane.coeffs = stair.vPlanes[Q].coeffs;
            stair.vLevels.push_back(temp_plane);

            max_level++;
          }
        }
      }
    }
  }

  if (max_level < 1)
    are_stairs = false;

  return are_stairs;
}
