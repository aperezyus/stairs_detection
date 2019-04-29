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

#ifndef STAIR_CLASSES_H
#define STAIR_CLASSES_H

#include <boost/filesystem.hpp>
#include <pcl/PCLHeader.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common_headers.h>

#include "RGBD/plane.h"

int neighbour_search(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointXYZ point, float radius, pcl::PointCloud<pcl::PointXYZ>::Ptr &neighbouring_cloud);
int neighbour_search(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointXYZ point, float radius);

//struct Step
//{
//	// Constructor with standard settings
//	Step()
//	{
   
//	}
	

	
//	//~ int index;
//	//~ float D;
//	//~ float width;
//	//~ float length;
//	//~ int n_points;
//	//~ float area;
//	//~ int level;
//	//~ Matrix3f dir;
//	//~ float solidity;
//	//~ pcl::PointXYZ center;
	
//	public:
//	  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
//};


//struct Level
//{
//	// Constructor with standard settings
//	Level()
//	{
   
//	}
	
//	//~ int level;
//	//~ int type; // 0 = step, 1 = floor, 2 = virtual;
//	//~ pcl::PointXYZ waypoint;
//	//~ bool walkable;
//	//~ std::vector<int> index;
//	//~ float D;
//	//~ float biggest_area;
//	//~ int biggest_index;
//	//~ Matrix3f dir; // .col(2) = width dir, .col(1) = length dir.
//	//~ pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
//	//~ pcl::PointXYZ centroid;
//	//~ pcl::PointXYZ center;
//	//~
//	//~ float width;
//	//~ float length;
//	//~ float area;
//	//~ float solidity;
	
//	public:
//	  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
//};

Plane getBiggestStep (std::vector<Plane> vPlanes);

struct Stair
{
	// Constructor with standard settings
	Stair()
	{
   
	}
	
	~Stair()
	{
		
	}
	
  void getStairDirFromPlane(Eigen::Affine3d c2f, Plane &plane);
  Plane getBestStep (Eigen::Affine3d T);
	void getInitialStepVertices();
	void getInitialStepVertices(Eigen::Matrix3f & dir);
	// void getStepVertices(Eigen::Matrix3f c2m);
	void getClimbingStepVertices(Eigen::Affine3d & c2m);
	void getInitialStairVolume(Eigen::Matrix3f manhattan_dirs, Eigen::Matrix3f standalone_dirs, bool has_manhattan);
	void getExactStepVertices();
	// old
	// void getInitialStepVertices(Eigen::Matrix3f c2m);
	void getInitialStairVolume();
	
	std::vector<Plane> vPlanes;
	std::vector<Plane> vLevels;
	std::vector<Plane> vOrientedLevels;
	std::vector<Plane> vRisers;
	Eigen::Matrix3f stair_dir;
	//~ Eigen::Matrix4f i2s;
	//~ Eigen::Matrix4f s2i;
	Eigen::Affine3d i2s;
	Eigen::Affine3d i2s_non_corrected;
	Eigen::Affine3d s2i;
	pcl::PointXYZ centroid;
	pcl::PointXYZ center;
	
	//~ Matrix3f dir;
	float width;
	float length;
	float height;
	
	float step_width;
	float step_length;
	float step_height;

	std::string type;
	
	pcl::PointXYZ initial_point;
	//~ pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
	//~ int best_level;
	//~ Vector3f back_dir;

	int n_steps;
	int step_im_on;

	public:
	  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif
