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
//#include "custom_functions.h"

Plane getBiggestStep (std::vector<Plane> vPlanes)
{
  int index = 0;
  int size = 0;

  for (int Q = 0; Q<vPlanes.size(); Q++)
  {

    if (Q == 0)
    {
      size = vPlanes[Q].cloud->points.size();
    }
    else
    {
      // if (vPlanes[Q].level > 0)
//      if (vPlanes[Q].isValid)
//      {
        int current_size = vPlanes[Q].cloud->points.size();
        if (current_size > size)
        {
          index = Q;
          size = current_size;
        }
//      }
    }


  }
  return vPlanes[index];
}

void Stair::getStairDirFromPlane(Eigen::Affine3d c2f, Plane &plane)
{
  if (plane.eigDx.isZero(0))
      plane.getPrincipalDirections();

  Eigen::Vector3f best_normal2c = plane.eigDx.col(2);
  Eigen::Vector3f best_normal = c2f.rotation().cast<float>()*best_normal2c;

  float angle_x = acos(Eigen::Vector2f(best_normal(0),best_normal(2)).normalized().dot(Eigen::Vector2f(1,0)));
  float angle_z = acos(Eigen::Vector2f(best_normal(0),best_normal(2)).normalized().dot(Eigen::Vector2f(0,1)));
//    float angle_x = getHorizontalAngle(best_normal,Eigen::Vector3f(1,0,0));
//    float angle_z = getHorizontalAngle(best_normal,Eigen::Vector3f(0,0,1));

  float threshold = 45*M_PI/180;

  Eigen::Matrix3f eigDx2f = Eigen::Matrix3f::Identity();

  if ((fabs(2*M_PI-angle_z) < threshold) or (angle_z < threshold))
  {
      eigDx2f.col(0) = Eigen::Vector3f(best_normal(2),0, -best_normal(0));
      eigDx2f.col(2) = Eigen::Vector3f(best_normal(0),0,best_normal(2));
      eigDx2f.col(1) = eigDx2f.col(2).cross(eigDx2f.col(0));
  }
  else if (fabs(M_PI-angle_z) < threshold)
  {
      eigDx2f.col(0) = Eigen::Vector3f(-best_normal(2),0,best_normal(0));
      eigDx2f.col(2) = Eigen::Vector3f(-best_normal(0),0,-best_normal(2));
      eigDx2f.col(1) = eigDx2f.col(2).cross(eigDx2f.col(0));
  }
  else
  {
      if ((fabs(2*M_PI-angle_x) < threshold) or (angle_x < threshold))
      {
          eigDx2f.col(0) = Eigen::Vector3f(best_normal(0),0, best_normal(2));
          eigDx2f.col(2) = Eigen::Vector3f(-best_normal(2),0, best_normal(0));
          eigDx2f.col(1) = eigDx2f.col(2).cross(eigDx2f.col(0));

      }
      else if (fabs(M_PI-angle_x) < threshold)
      {
          eigDx2f.col(0) = Eigen::Vector3f(-best_normal(0),0,-best_normal(2));
          eigDx2f.col(2) = Eigen::Vector3f(best_normal(2),0,-best_normal(0));
          eigDx2f.col(1) = eigDx2f.col(2).cross(eigDx2f.col(0));

      }
      else
      {
          eigDx2f.col(0) = Eigen::Vector3f(best_normal(0),0,best_normal(2));
          eigDx2f.col(2) = Eigen::Vector3f(-best_normal(2),0,best_normal(0));
          eigDx2f.col(1) = eigDx2f.col(2).cross(eigDx2f.col(0));
      }
  }

  Eigen::Affine3f f2c_aux = c2f.inverse().cast<float>();
  f2c_aux.translation() = Eigen::Vector3f(0,0,0);

  Eigen::Matrix3f step_dir;

  step_dir.col(0) = (f2c_aux)*eigDx2f.col(0);
  step_dir.col(1) = (f2c_aux)*eigDx2f.col(1);
  step_dir.col(2) = (f2c_aux)*eigDx2f.col(2);

  plane.eigDx = step_dir;
}

Plane Stair::getBestStep(Eigen::Affine3d T)
{
  int index = 0;
  float max_extent = 0;

  float sum_of_contour_areas = 0;

// std::cout << "Sum of contour areas" << std::endl;
  for (int Q = 1; Q<vLevels.size(); Q++)
  {
    // std::cout << "Q = " << Q << std::endl;
    // if (vPlanes[Q].contour_area == 0)
      sum_of_contour_areas += vLevels[Q].getContourArea();
    // else
    // 	sum_of_contour_areas += vPlanes[Q].contour_area;
      // std::cout << "suman " << sum_of_contour_areas << std::endl;
  }

  // std::cout << "Sum of contour areas: " << sum_of_contour_areas << std::endl;
  // std::cout << "Planos a comprobar: " << std::endl;
  for (int Q = 1; Q<vLevels.size(); Q++)
  {
    // std::cout << "Q = " << Q << std::endl;
    if (vLevels[Q].eigDx.isZero(0))
      vLevels[Q].getPrincipalDirections();

    getStairDirFromPlane(T,vLevels[Q]);

//    vLevels[Q].getStairDirFromPCA(T);

    // std::cout << "Sum of rectangle areas" << std::endl;

    float sum_of_rectangle_areas = 0;
    for (int P = 1; P<vLevels.size(); P++)
    {
      // std::cout << "P = " << P << std::endl;
      sum_of_rectangle_areas += vLevels[P].getRectangleArea(vLevels[Q].eigDx);
      // std::cout << "suman " << sum_of_rectangle_areas << std::endl;
    }

    // std::cout << "Sum of rectangle areas: " << sum_of_rectangle_areas << std::endl;

    float extent = sum_of_contour_areas/sum_of_rectangle_areas;
    // std::cout << "EXTENT: " << extent << std::endl;
    if (extent > max_extent)
    {
      max_extent = extent;
      index = Q;
    }

    // std::cout << "Best step is " << index << std::endl;
  }
  return vLevels[index];
}


void Stair::getInitialStepVertices()
{
	Eigen::Matrix3f aux = stair_dir;
	
	for (int Q=1; Q<vLevels.size(); Q++)
	{
		// vLevels[Q].getPrincipalDirections();
		vLevels[Q].getMeasurements(stair_dir);
		
		if (vLevels[Q].width < vLevels[Q].length)
		{
			// std::cout << "anchura menor que largura " << Q<< std::endl;
			stair_dir.col(0) = aux.col(2);
			stair_dir.col(2) = -aux.col(0);
			
			vLevels[Q].getMeasurements(stair_dir);
		}
		
		vLevels[Q].getVertices(stair_dir);
		
		if (Q == 2)
		{
//			if (getAngle(stair_dir.col(2),Eigen::Vector3f(vLevels[Q].vertices->points[4].x-vLevels[Q-1].vertices->points[4].x,0,vLevels[Q].vertices->points[4].z-vLevels[Q-1].vertices->points[4].z))*180/M_PI > 90)
      if (acos(stair_dir.col(2).dot(Eigen::Vector3f(vLevels[Q].vertices->points[4].x-vLevels[Q-1].vertices->points[4].x,0,vLevels[Q].vertices->points[4].z-vLevels[Q-1].vertices->points[4].z).normalized()))>M_PI_2)
			{
				
				// std::cout << "estos ejes van al revés!" << std::endl;
				
				stair_dir.col(0) = -stair_dir.col(0);
				stair_dir.col(2) = -stair_dir.col(2);
				
				vLevels[Q].getVertices(stair_dir);
				vLevels[Q-1].getVertices(stair_dir);
			}
		}

		// std::cout << vLevels[Q].vertices->points[0].x << " " << vLevels[Q].vertices->points[0].y << " " << vLevels[Q].vertices->points[0].z << std::endl;
		// std::cout << vLevels[Q].vertices->points[1].x << " " << vLevels[Q].vertices->points[1].y << " " << vLevels[Q].vertices->points[1].z << std::endl;
		// std::cout << vLevels[Q].vertices->points[2].x << " " << vLevels[Q].vertices->points[2].y << " " << vLevels[Q].vertices->points[2].z << std::endl;
		// std::cout << vLevels[Q].vertices->points[3].x << " " << vLevels[Q].vertices->points[3].y << " " << vLevels[Q].vertices->points[3].z << std::endl;
		// std::cout << vLevels[Q].vertices->points[4].x << " " << vLevels[Q].vertices->points[4].y << " " << vLevels[Q].vertices->points[4].z << std::endl << std::endl;
	}
}

void Stair::getInitialStepVertices(Eigen::Matrix3f & dir)
{
	Eigen::Matrix3f aux = dir;
	
	for (int Q=1; Q<vLevels.size(); Q++)
	{
		// vLevels[Q].getPrincipalDirections();
		vLevels[Q].getMeasurements(dir);
		
		if (Q == 1)
		{
			if (vLevels[Q].width < vLevels[Q].length)
			{
//        std::cout << "anchura menor que largura " << Q<< std::endl;
				dir.col(0) = aux.col(2);
				dir.col(2) = -aux.col(0);
				
				vLevels[Q].getMeasurements(dir);
			}
		}
		
		
		vLevels[Q].getVertices(dir);
		
		if (Q == 2)
		{
      if (acos(stair_dir.col(2).dot(Eigen::Vector3f(vLevels[Q].vertices->points[4].x-vLevels[Q-1].vertices->points[4].x,0,vLevels[Q].vertices->points[4].z-vLevels[Q-1].vertices->points[4].z).normalized()))>M_PI_2)
//			if (getAngle(dir.col(2),Eigen::Vector3f(vLevels[Q].vertices->points[4].x-vLevels[Q-1].vertices->points[4].x,0,vLevels[Q].vertices->points[4].z-vLevels[Q-1].vertices->points[4].z))*180/M_PI > 90)
			{
				
//        std::cout << "estos ejes van al revés!" << std::endl;
				
				dir.col(0) = -dir.col(0);
				dir.col(2) = -dir.col(2);
				
				vLevels[Q].getVertices(dir);
				vLevels[Q-1].getVertices(dir);
			}
		}

//     std::cout << vLevels[Q].vertices->points[0].x << " " << vLevels[Q].vertices->points[0].y << " " << vLevels[Q].vertices->points[0].z << std::endl;
//     std::cout << vLevels[Q].vertices->points[1].x << " " << vLevels[Q].vertices->points[1].y << " " << vLevels[Q].vertices->points[1].z << std::endl;
//     std::cout << vLevels[Q].vertices->points[2].x << " " << vLevels[Q].vertices->points[2].y << " " << vLevels[Q].vertices->points[2].z << std::endl;
//     std::cout << vLevels[Q].vertices->points[3].x << " " << vLevels[Q].vertices->points[3].y << " " << vLevels[Q].vertices->points[3].z << std::endl;
//     std::cout << vLevels[Q].vertices->points[4].x << " " << vLevels[Q].vertices->points[4].y << " " << vLevels[Q].vertices->points[4].z << std::endl << std::endl;
	}
}

// void Stair::getStepVertices(Eigen::Matrix3f c2m)
// {
// 	stair_dir = c2m;
	
// 	for (int Q=0; Q<vLevels.size(); Q++)
// 	{
// 		// vLevels[Q].getPrincipalDirections();
// 		vLevels[Q].getMeasurements(stair_dir);
		
// 		if (vLevels[Q].width < vLevels[Q].length)
// 		{
// 			std::cout << "anchura menor que largura " << Q<< std::endl;
// 			stair_dir.col(0) = c2m.col(2);
// 			stair_dir.col(2) = -c2m.col(0);
			
// 			vLevels[Q].getMeasurements(stair_dir);
// 		}
		
// 		vLevels[Q].getVertices(stair_dir);
		
// 		if (Q == 2)
// 		{
// 			if (getAngle(stair_dir.col(2),Eigen::Vector3f(vLevels[Q].vertices->points[4].x-vLevels[Q-1].vertices->points[4].x,0,vLevels[Q].vertices->points[4].z-vLevels[Q-1].vertices->points[4].z))*180/M_PI > 90)
// 			{
				
// 				std::cout << "estos ejes van al revés!" << std::endl;
				
// 				stair_dir.col(0) = -stair_dir.col(0);
// 				stair_dir.col(2) = -stair_dir.col(2);
				
// 				vLevels[Q].getVertices(stair_dir);
// 				vLevels[Q-1].getVertices(stair_dir);
// 			}
// 		}

// 		std::cout << vLevels[Q].vertices->points[0].x << " " << vLevels[Q].vertices->points[0].y << " " << vLevels[Q].vertices->points[0].z << std::endl;
// 		std::cout << vLevels[Q].vertices->points[1].x << " " << vLevels[Q].vertices->points[1].y << " " << vLevels[Q].vertices->points[1].z << std::endl;
// 		std::cout << vLevels[Q].vertices->points[2].x << " " << vLevels[Q].vertices->points[2].y << " " << vLevels[Q].vertices->points[2].z << std::endl;
// 		std::cout << vLevels[Q].vertices->points[3].x << " " << vLevels[Q].vertices->points[3].y << " " << vLevels[Q].vertices->points[3].z << std::endl;
// 		std::cout << vLevels[Q].vertices->points[4].x << " " << vLevels[Q].vertices->points[4].y << " " << vLevels[Q].vertices->points[4].z << std::endl << std::endl;
// 	}
// }

void Stair::getClimbingStepVertices(Eigen::Affine3d & c2m)
{
	stair_dir = c2m.rotation().transpose().cast<float>();
	// stair_dir = c2m.rotation().cast<float>();
	bool correction = false;
	
	for (int Q=0; Q<vLevels.size(); Q++)
	{
		vLevels[Q].getMeasurements(c2m);
		
		// if ((vLevels[Q].width < vLevels[Q].length) and (!correction))
		// {
		// 	std::cout << "anchura menor que largura " << Q<< std::endl;
		// 	stair_dir.col(0) = c2m.rotation().transpose().cast<float>().col(2);
		// 	stair_dir.col(2) = -c2m.rotation().transpose().cast<float>().col(0);
			
		// 	correction = true;
		// 	c2m = Eigen::Translation3d(c2m.translation()) * Eigen::AngleAxisd(stair_dir.transpose().cast<double>());
		// 	vLevels[Q].getMeasurements(c2m);
		// }

		vLevels[Q].getVertices(stair_dir);
		
		if (Q == 2)
		{
			// std::cout << stair_dir.col(2) << std::endl;
			// std::cout << Eigen::Vector3f(vLevels[Q].vertices->points[4].x-vLevels[Q-1].vertices->points[4].x,0,	vLevels[Q].vertices->points[4].z-vLevels[Q-1].vertices->points[4].z) << std::endl;
			// std::cout << getHorizontalAngle(stair_dir.col(2),Eigen::Vector3f(vLevels[Q].vertices->points[4].x-vLevels[Q-1].vertices->points[4].x,0,vLevels[Q].vertices->points[4].z-vLevels[Q-1].vertices->points[4].z))*180/M_PI << std::endl << std::endl;
//			if (getAngle(stair_dir.col(2),Eigen::Vector3f(vLevels[Q].vertices->points[4].x-vLevels[Q-1].vertices->points[4].x,0,
//					vLevels[Q].vertices->points[4].z-vLevels[Q-1].vertices->points[4].z))*180/M_PI > 90)
      if (acos(stair_dir.col(2).dot(Eigen::Vector3f(vLevels[Q].vertices->points[4].x-vLevels[Q-1].vertices->points[4].x,0,vLevels[Q].vertices->points[4].z-vLevels[Q-1].vertices->points[4].z).normalized()))>M_PI_2)
      {
				// std::cout << "estos ejes van al revés!" << std::endl; 
				stair_dir.col(0) = -stair_dir.col(0);
				stair_dir.col(2) = -stair_dir.col(2);

				vLevels[Q].getVertices(stair_dir);
				vLevels[Q-1].getVertices(stair_dir);
			}
		}

		// std::cout << vLevels[Q].centroid2f.getVector3fMap() << std::endl;

		// std::cout << vLevels[Q].vertices->points[0].x << " " << vLevels[Q].vertices->points[0].y << " " << vLevels[Q].vertices->points[0].z << std::endl;
		// std::cout << vLevels[Q].vertices->points[1].x << " " << vLevels[Q].vertices->points[1].y << " " << vLevels[Q].vertices->points[1].z << std::endl;
		// std::cout << vLevels[Q].vertices->points[2].x << " " << vLevels[Q].vertices->points[2].y << " " << vLevels[Q].vertices->points[2].z << std::endl;
		// std::cout << vLevels[Q].vertices->points[3].x << " " << vLevels[Q].vertices->points[3].y << " " << vLevels[Q].vertices->points[3].z << std::endl;
		// std::cout << vLevels[Q].vertices->points[4].x << " " << vLevels[Q].vertices->points[4].y << " " << vLevels[Q].vertices->points[4].z << std::endl << std::endl;
	}

	c2m = Eigen::Translation3d(c2m.translation()) * Eigen::AngleAxisd(stair_dir.transpose().cast<double>());
	// c2m = Eigen::Translation3d(c2m.translation()) * Eigen::AngleAxisd(stair_dir.cast<double>());
}

void Stair::getInitialStairVolume(Eigen::Matrix3f manhattan_dirs, Eigen::Matrix3f standalone_dirs, bool has_manhattan)
{

	pcl::PointCloud<pcl::PointXYZ>::Ptr allPoints (new pcl::PointCloud<pcl::PointXYZ>);
	
	for (int Q=1; Q<vLevels.size(); Q++)
	{
		*allPoints += *vLevels[Q].cloud;
	}
	Eigen::Vector4f vector_centroid;
	pcl::compute3DCentroid(*allPoints,vector_centroid);
	centroid = pcl::PointXYZ (vector_centroid[0], vector_centroid[1], vector_centroid[2]);

	
	this->getInitialStepVertices(standalone_dirs);
	Eigen::Matrix3d rotation;
    Eigen::Matrix3d rotation_standalone = standalone_dirs.transpose().cast<double>();
    Eigen::Vector3d translation;
    Eigen::Vector3d translation_standalone = -1 * (rotation_standalone * centroid.getVector3fMap().cast<double>());
    Eigen::Affine3d i2s_standalone = ( Eigen::Translation3d (translation_standalone) * Eigen::AngleAxisd (rotation_standalone));
    pcl::PointCloud<pcl::PointXYZ> cPoints_standalone;
    pcl::transformPointCloud(*allPoints, cPoints_standalone, i2s_standalone);

	
    pcl::PointXYZ min_pt, max_pt;
    pcl::PointXYZ min_pt_standalone, max_pt_standalone;
    
    pcl::getMinMax3D(cPoints_standalone, min_pt_standalone, max_pt_standalone);
    
    float width_standalone = fabs(max_pt_standalone.x - min_pt_standalone.x);
    float height_standalone = fabs(max_pt_standalone.y - min_pt_standalone.y);
    float length_standalone = fabs(max_pt_standalone.z - min_pt_standalone.z);

//    std::cout << width_standalone << " " << height_standalone << " " << length_standalone << std::endl;
    

    float volume_standalone = width_standalone*height_standalone*length_standalone;
    



    if (has_manhattan)
    {
		this->getInitialStepVertices(manhattan_dirs);
		Eigen::Matrix3d rotation_manhattan = manhattan_dirs.transpose().cast<double>();
		Eigen::Vector3d translation_manhattan = -1 * (rotation_manhattan * centroid.getVector3fMap().cast<double>());
		Eigen::Affine3d i2s_manhattan = ( Eigen::Translation3d (translation_manhattan) * Eigen::AngleAxisd (rotation_manhattan));
		pcl::PointCloud<pcl::PointXYZ> cPoints_manhattan;
	    pcl::transformPointCloud(*allPoints, cPoints_manhattan, i2s_manhattan);
	    pcl::PointXYZ min_pt_manhattan, max_pt_manhattan;
	    pcl::getMinMax3D(cPoints_manhattan, min_pt_manhattan, max_pt_manhattan);

	    float width_manhattan = fabs(max_pt_manhattan.x - min_pt_manhattan.x);
	    float height_manhattan = fabs(max_pt_manhattan.y - min_pt_manhattan.y);
	    float length_manhattan = fabs(max_pt_manhattan.z - min_pt_manhattan.z);

//      std::cout << width_manhattan << " " << height_manhattan << " " << length_manhattan << std::endl;

	    float volume_manhattan = width_manhattan*height_manhattan*length_manhattan;

//      std::cout << "Standalone " << volume_standalone << std::endl;
//      std::cout << "Manhattan " << volume_manhattan << std::endl;

	    if (volume_standalone < 0.95*volume_manhattan)
	    // if (volume_standalone < volume_manhattan)
	    {
	    	
	    	width = width_standalone;
	    	length = length_standalone;
	    	height = height_standalone;

	    	i2s = i2s_standalone;
	    	stair_dir = standalone_dirs;
	    	rotation = rotation_standalone;
	    	translation = translation_standalone;

	    	max_pt = max_pt_standalone;
	    	min_pt = min_pt_standalone;
	    }
	    else
	    {
	    	width = width_manhattan;
	    	length = length_manhattan;
	    	height = height_manhattan;

	    	i2s = i2s_manhattan;
	    	stair_dir = manhattan_dirs;
	    	rotation = rotation_manhattan;
	    	translation = translation_manhattan;

	    	max_pt = max_pt_manhattan;
	    	min_pt = min_pt_manhattan;
	    }
	}
	else
	{
		width = width_standalone;
    	length = length_standalone;
    	height = height_standalone;

    	i2s = i2s_standalone;
    	stair_dir = standalone_dirs;
    	rotation = rotation_standalone;
    	translation = translation_standalone;

    	max_pt = max_pt_standalone;
    	min_pt = min_pt_standalone;
	}


    

    step_width = width;
    
    const Eigen::Vector3f mean_diag = 0.5f*(max_pt.getVector3fMap() + min_pt.getVector3fMap());
	Eigen::Vector3f mean_diag_absolute = stair_dir*mean_diag+ centroid.getVector3fMap().head<3>();
	center = pcl::PointXYZ (mean_diag_absolute[0], mean_diag_absolute[1], mean_diag_absolute[2]);

	Eigen::Vector3f initial_point_vector;
	// std::cout << min_pt.x << std::endl;
	// std::cout << min_pt.y << std::endl;
	// std::cout << min_pt.z << std::endl << std::endl;

	// std::cout << max_pt.x << std::endl;
	// std::cout << max_pt.y << std::endl;
	// std::cout << max_pt.z << std::endl << std::endl;

	if (type == "down")
		initial_point_vector = mean_diag_absolute - length/2*stair_dir.col(2)  + height/2*stair_dir.col(1);
	else
		initial_point_vector = mean_diag_absolute - length/2*stair_dir.col(2)  - height/2*stair_dir.col(1);

  initial_point = pcl::PointXYZ(initial_point_vector(0),initial_point_vector(1),initial_point_vector(2));
//  translation = -1 * (rotation * initial_point_vector.cast<double>());

//    i2s = ( Eigen::Translation3d (translation) * Eigen::AngleAxisd (rotation));
//     s2i = i2s.inverse();
//	s2i = createAffine3d(stair_dir,initial_point_vector);
    s2i = ( Eigen::Translation3d (initial_point_vector.cast<double>()) * Eigen::AngleAxisd (stair_dir.cast<double>()));
    i2s = s2i.inverse();


}

void Stair::getExactStepVertices()
{
	
	float cumulated_height = 0;
	int n_height = 0;
	float cumulated_length = 0;
	int n_length = 0;
	
	for (int Q=0; Q<vLevels.size(); Q++)
	{
		if ((type == "down") and (Q == 1) and (vOrientedLevels[0].vertices->points.size() > 0))
		{
			Eigen::Matrix3d rotation_i2s = i2s.rotation();
			Eigen::Matrix3d rotation_s2i = s2i.rotation();
			Eigen::Vector3f first_step_centroid2s;
			pcl::transformPoint(vLevels[1].centroid.getVector3fMap(),first_step_centroid2s,i2s.cast<float>());
			float centroid_correction = first_step_centroid2s(1);
			Eigen::Vector3f initial_vector_point_aux = Eigen::Vector3f(0,centroid_correction,vOrientedLevels[0].vertices->points[0].z);
			pcl::transformPoint(initial_vector_point_aux,initial_vector_point_aux, s2i.cast<float>());	
			Eigen::Vector3d translation = -1 * (rotation_i2s * initial_vector_point_aux.cast<double>());

      i2s = ( Eigen::Translation3d (translation) * Eigen::AngleAxisd (rotation_i2s));
			s2i = i2s.inverse();
			stair_dir = s2i.rotation().cast<float>();

			for (int Q = 1; Q<vLevels.size(); Q++)
			{
        vLevels[Q].getVertices(stair_dir);
			}
		}
		else if ((type == "up") and (Q ==1))
		{
			Eigen::Matrix3d rotation_i2s = i2s.rotation();
			Eigen::Matrix3d rotation_s2i = s2i.rotation();
			Eigen::Vector3f first_step_centroid2s;
			pcl::transformPoint(vLevels[1].centroid.getVector3fMap(),first_step_centroid2s,i2s.cast<float>());
			float centroid_correction = first_step_centroid2s(1);
			Eigen::Vector3f initial_vector_point_aux = Eigen::Vector3f(0,centroid_correction,0);
			pcl::transformPoint(initial_vector_point_aux,initial_vector_point_aux, s2i.cast<float>());	
			Eigen::Vector3d translation = -1 * (rotation_i2s * initial_vector_point_aux.cast<double>());

      i2s = ( Eigen::Translation3d (translation) * Eigen::AngleAxisd (rotation_i2s));
			s2i = i2s.inverse();
			stair_dir = s2i.rotation().cast<float>();

			for (int Q = 1; Q<vLevels.size(); Q++)
			{
        vLevels[Q].getVertices(stair_dir);
			}
		}
		if ((Q == 0) and (vLevels[0].cloud->points.size() > 0))
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr neighbouring_cloud (new pcl::PointCloud<pcl::PointXYZ>);
			if (neighbour_search(vLevels[0].cloud, initial_point, 0.5f, neighbouring_cloud) > 5)
			{				


				vLevels[Q].cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
				*vLevels[Q].cloud = *neighbouring_cloud;
				vLevels[Q].getCentroid();
				vLevels[Q].getMeasurements(stair_dir,neighbouring_cloud);
				vLevels[Q].width = step_width;
				vLevels[Q].getVertices(stair_dir);
				
				Plane temp_plane;
				temp_plane.vertices.reset(new pcl::PointCloud<pcl::PointXYZ>);
				
				// move the points to the that reference frame
				
				pcl::transformPointCloud(*vLevels[Q].vertices, *temp_plane.vertices, i2s);
//        *temp_plane.vertices += *vLevels[Q].vertices;
				vOrientedLevels.push_back(temp_plane);
			}
			else
			{
				Plane temp_plane;
				temp_plane.vertices.reset(new pcl::PointCloud<pcl::PointXYZ>);
				vOrientedLevels.push_back(temp_plane);
			}			
		}
		else if (Q != 0)
		{
			
			vLevels[Q].width = step_width;
		
			Plane temp_plane;
			temp_plane.vertices.reset(new pcl::PointCloud<pcl::PointXYZ>);
			
			// move the points to the that reference frame
			pcl::transformPointCloud(*vLevels[Q].vertices, *temp_plane.vertices, i2s);
//			*temp_plane.vertices += *vLevels[Q].vertices;
			vOrientedLevels.push_back(temp_plane);

		}
		else
		{
			Plane temp_plane;
			temp_plane.vertices.reset(new pcl::PointCloud<pcl::PointXYZ>);
			vOrientedLevels.push_back(temp_plane);
		}
		
//     if (vOrientedLevels[Q].vertices->points.size() > 0)
//     {
//      std::cout << Q << std::endl;
//      std::cout << vOrientedLevels[Q].vertices->points[0].x << " " << vOrientedLevels[Q].vertices->points[0].y << " " << vOrientedLevels[Q].vertices->points[0].z << std::endl;
//      std::cout << vOrientedLevels[Q].vertices->points[1].x << " " << vOrientedLevels[Q].vertices->points[1].y << " " << vOrientedLevels[Q].vertices->points[1].z << std::endl;
//      std::cout << vOrientedLevels[Q].vertices->points[2].x << " " << vOrientedLevels[Q].vertices->points[2].y << " " << vOrientedLevels[Q].vertices->points[2].z << std::endl;
//      std::cout << vOrientedLevels[Q].vertices->points[3].x << " " << vOrientedLevels[Q].vertices->points[3].y << " " << vOrientedLevels[Q].vertices->points[3].z << std::endl;
//      std::cout << vOrientedLevels[Q].vertices->points[4].x << " " << vOrientedLevels[Q].vertices->points[4].y << " " << vOrientedLevels[Q].vertices->points[4].z << std::endl << std::endl;
//     }


		if ((Q>1) or ((Q == 1) and (vOrientedLevels[0].vertices->points.size() > 0)))
		{
			// HEIGHT CORRECTION
			n_height++;
			cumulated_height += vOrientedLevels[Q-1].vertices->points[0].y - vOrientedLevels[Q].vertices->points[0].y;
//      std::cout << n_height << " " << cumulated_height << std::endl;
//      std::cout << vOrientedLevels[Q-1].vertices->points[0].y - vOrientedLevels[Q].vertices->points[0].y << std::endl;
//      std::cout << vOrientedLevels[Q-1].vertices->points[0].y << std::endl;
//      std::cout << vOrientedLevels[Q-1].vertices->points[4].y << std::endl;

//      std::cout << vOrientedLevels[Q].vertices->points[0].y << std::endl;
//      std::cout << vOrientedLevels[Q].vertices->points[4].y << std::endl;


			
			// LENGTH CORRECTION
			// if (vOrientedLevels[Q-1].vertices->points[0].y > vOrientedLevels[Q].vertices->points[0].y) // escaleras de bajada
			if (type == "down")
			{
				// if (Q == 1)
				// {
				// 	Eigen::Matrix3d rotation_i2s = i2s.rotation();
				// 	Eigen::Matrix3d rotation_s2i = s2i.rotation();
				// 	pcl::PointXYZ initial_point_aux = pcl::PointXYZ(0,0,vOrientedLevels[0].vertices->points[0].z);
				// 	Eigen::Vector3f initial_vector_point_aux = Eigen::Vector3f(0,0,vOrientedLevels[0].vertices->points[0].z);
				// 	pcl::transformPoint(initial_vector_point_aux,initial_vector_point_aux, s2i.cast<float>());	
				// 	Eigen::Vector3d translation = -1 * (rotation_i2s * initial_vector_point_aux.cast<double>());

				//     i2s = ( Eigen::Translation3d (translation) * Eigen::AngleAxisd (rotation_i2s));
				// 	s2i = i2s.inverse();
				// }
				vOrientedLevels[Q].vertices->points[2].z = vOrientedLevels[Q-1].vertices->points[0].z;
				vOrientedLevels[Q].vertices->points[3].z = vOrientedLevels[Q-1].vertices->points[0].z;
				vOrientedLevels[Q].vertices->points[4].z = (vOrientedLevels[Q].vertices->points[0].z+vOrientedLevels[Q].vertices->points[2].z)/2;
				
				n_length++;
				cumulated_length += vOrientedLevels[Q].vertices->points[0].z - vOrientedLevels[Q].vertices->points[2].z;
			}
			else // escaleras de subida
			{
				if (Q>1)
				{
					vOrientedLevels[Q-1].vertices->points[0].z = vOrientedLevels[Q].vertices->points[2].z;
					vOrientedLevels[Q-1].vertices->points[1].z = vOrientedLevels[Q].vertices->points[3].z;
					vOrientedLevels[Q-1].vertices->points[4].z = (vOrientedLevels[Q].vertices->points[0].z+vOrientedLevels[Q].vertices->points[2].z)/2;
					
					n_length++;
					cumulated_length += vOrientedLevels[Q-1].vertices->points[0].z - vOrientedLevels[Q-1].vertices->points[2].z;
				}
			}
		}
		// WIDTH CORRECTION
		if (vOrientedLevels[Q].vertices->points.size() > 0)
		{
			vOrientedLevels[Q].vertices->points[0].x = width/2;
			vOrientedLevels[Q].vertices->points[1].x = -width/2;
			vOrientedLevels[Q].vertices->points[2].x = -width/2;
			vOrientedLevels[Q].vertices->points[3].x = width/2;
			vOrientedLevels[Q].vertices->points[4].x = 0;
		}
	}
	
	step_height = fabs(cumulated_height/n_height);
	if (n_length > 0)
	{
		step_length = cumulated_length/n_length;


	
		// Correction of vertices in case of first step and no floor in descending staircase or last step in ascending staircase
		if (vOrientedLevels.size()>2)
		{
			if ((vOrientedLevels[0].vertices->points.size() == 0) and (vOrientedLevels[2].vertices->points[4].y < vOrientedLevels[1].vertices->points[4].y))
			{
				//~ std::cout << "Corrección primer escalón en escaleras de bajada" << std::endl;
				vOrientedLevels[1].vertices->points[2].z = vOrientedLevels[1].vertices->points[0].z-step_length;
				vOrientedLevels[1].vertices->points[3].z = vOrientedLevels[1].vertices->points[0].z-step_length;
				vOrientedLevels[1].vertices->points[4].z = (vOrientedLevels[1].vertices->points[0].z+vOrientedLevels[1].vertices->points[2].z)/2;
			}
			if (vOrientedLevels[2].vertices->points[4].y > vOrientedLevels[1].vertices->points[4].y)
			{
				//~ std::cout << "Corrección último escalón en escaleras de subida " << std::endl;
				vOrientedLevels[vOrientedLevels.size()-1].vertices->points[0].z = vOrientedLevels[vOrientedLevels.size()-1].vertices->points[2].z + step_length;
				vOrientedLevels[vOrientedLevels.size()-1].vertices->points[1].z = vOrientedLevels[vOrientedLevels.size()-1].vertices->points[2].z + step_length;
				vOrientedLevels[vOrientedLevels.size()-1].vertices->points[4].z = (vOrientedLevels[vOrientedLevels.size()-1].vertices->points[0].z+vOrientedLevels[vOrientedLevels.size()-1].vertices->points[2].z)/2;
			}
			
		}
	}

	


	Plane empty_plane;
	vRisers.push_back(empty_plane);
	
	for (int Q = 1; Q< vOrientedLevels.size(); Q++)
	{				
		Plane temp_plane;
		temp_plane.vertices.reset(new pcl::PointCloud<pcl::PointXYZ>);
		if (Q == 1)
		{

			temp_plane.vertices->points.push_back(vOrientedLevels[Q].vertices->points[3]);
			temp_plane.vertices->points.push_back(vOrientedLevels[Q].vertices->points[2]);
			temp_plane.vertices->points.push_back(pcl::PointXYZ(vOrientedLevels[Q].vertices->points[2].x,vOrientedLevels[Q].vertices->points[2].y-step_height,vOrientedLevels[Q].vertices->points[2].z));
			temp_plane.vertices->points.push_back(pcl::PointXYZ(vOrientedLevels[Q].vertices->points[3].x,vOrientedLevels[Q].vertices->points[3].y-step_height,vOrientedLevels[Q].vertices->points[3].z));
			
			temp_plane.vertices->points.push_back(pcl::PointXYZ(0,vOrientedLevels[Q].vertices->points[2].y-step_height/2,vOrientedLevels[Q].vertices->points[2].z));
		}
		else if ((Q>1))// or ((Q == 1) and (vOrientedLevels[0].vertices->points.size() > 0)))
		{
			temp_plane.vertices->points.push_back(vOrientedLevels[Q-1].vertices->points[0]);
			temp_plane.vertices->points.push_back(vOrientedLevels[Q-1].vertices->points[1]);
			temp_plane.vertices->points.push_back(vOrientedLevels[Q].vertices->points[2]);
			temp_plane.vertices->points.push_back(vOrientedLevels[Q].vertices->points[3]);
			temp_plane.vertices->points.push_back(pcl::PointXYZ(0,(vOrientedLevels[Q].vertices->points[0].y+vOrientedLevels[Q-1].vertices->points[0].y)/2,vOrientedLevels[Q].vertices->points[0].z));
		}
		
		vRisers.push_back(temp_plane);

		pcl::transformPointCloud(*vRisers[Q].vertices,*vRisers[Q].vertices,s2i);
	}
	
  for (int Q = 1; Q< vOrientedLevels.size(); Q++)
	{		
		//~ vOrientedLevels[Q].vertices->points[0].y += 0.01f;
		//~ vOrientedLevels[Q].vertices->points[1].y += 0.01f;
		//~ vOrientedLevels[Q].vertices->points[2].y += 0.01f;
		//~ vOrientedLevels[Q].vertices->points[3].y += 0.01f;
		//~ vOrientedLevels[Q].vertices->points[4].y += 0.01f;

		pcl::transformPointCloud(*vOrientedLevels[Q].vertices,*vOrientedLevels[Q].vertices,s2i);
	}
    
    
//     std::cout << "Width :" << width << std::endl;
//     std::cout << "length :" << length << std::endl;
//     std::cout << "height :" << height << std::endl << std::endl;
    
//     std::cout << "Width :" << step_width << std::endl;
//     std::cout << "length :" << step_length << std::endl;
//     std::cout << "height :" << step_height << std::endl << std::endl;
}












// OLD
// void Stair::getInitialStepVertices(Eigen::Matrix3f c2m)
// {
// 	stair_dir = c2m;
	
// 	for (int Q=1; Q<vLevels.size(); Q++)
// 	{
// 		// vLevels[Q].getPrincipalDirections();
// 		vLevels[Q].getMeasurements(stair_dir);
		
// 		if (vLevels[Q].width < vLevels[Q].length)
// 		{
// 			// std::cout << "anchura menor que largura " << Q<< std::endl;
// 			stair_dir.col(0) = c2m.col(2);
// 			stair_dir.col(2) = -c2m.col(0);
			
// 			vLevels[Q].getMeasurements(stair_dir);
// 		}
		
// 		vLevels[Q].getVertices(stair_dir);
		
// 		if (Q == 2)
// 		{
// 			if (getAngle(stair_dir.col(2),Eigen::Vector3f(vLevels[Q].vertices->points[4].x-vLevels[Q-1].vertices->points[4].x,0,vLevels[Q].vertices->points[4].z-vLevels[Q-1].vertices->points[4].z))*180/M_PI > 90)
// 			{
				
// 				// std::cout << "estos ejes van al revés!" << std::endl;
				
// 				stair_dir.col(0) = -stair_dir.col(0);
// 				stair_dir.col(2) = -stair_dir.col(2);
				
// 				vLevels[Q].getVertices(stair_dir);
// 				vLevels[Q-1].getVertices(stair_dir);
// 			}
// 		}

// 		// std::cout << vLevels[Q].vertices->points[0].x << " " << vLevels[Q].vertices->points[0].y << " " << vLevels[Q].vertices->points[0].z << std::endl;
// 		// std::cout << vLevels[Q].vertices->points[1].x << " " << vLevels[Q].vertices->points[1].y << " " << vLevels[Q].vertices->points[1].z << std::endl;
// 		// std::cout << vLevels[Q].vertices->points[2].x << " " << vLevels[Q].vertices->points[2].y << " " << vLevels[Q].vertices->points[2].z << std::endl;
// 		// std::cout << vLevels[Q].vertices->points[3].x << " " << vLevels[Q].vertices->points[3].y << " " << vLevels[Q].vertices->points[3].z << std::endl;
// 		// std::cout << vLevels[Q].vertices->points[4].x << " " << vLevels[Q].vertices->points[4].y << " " << vLevels[Q].vertices->points[4].z << std::endl << std::endl;
// 	}
// }

void Stair::getInitialStairVolume()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr allVertices (new pcl::PointCloud<pcl::PointXYZ>);
	
	for (int Q=1; Q<vLevels.size(); Q++)
	{
		*allVertices += *vLevels[Q].vertices;
	}
	
	Eigen::Vector4f vector_centroid;
	pcl::compute3DCentroid(*allVertices,vector_centroid);
	centroid = pcl::PointXYZ (vector_centroid[0], vector_centroid[1], vector_centroid[2]);
	
	// move the points to the that reference frame
    //~ i2s = Eigen::Matrix4d::Identity();
    //~ i2s.block<3,3>(0,0) = stair_dir.transpose();
    Eigen::Matrix3d rotation = stair_dir.transpose().cast<double>();
    //~ i2s.rotation() = (stair_dir.transpose()).cast<double>();
    //~ i2s.block<3,1>(0,3) = -1.f * (i2s.block<3,3>(0,0) * centroid.getVector3fMap().head<3>());
    Eigen::Vector3d translation = -1 * (rotation * centroid.getVector3fMap().cast<double>());
    i2s = ( Eigen::Translation3d (translation) * Eigen::AngleAxisd (rotation));
    
    pcl::PointCloud<pcl::PointXYZ> cPoints;
    pcl::transformPointCloud(*allVertices, cPoints, i2s);
	
    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(cPoints, min_pt, max_pt);
    
    width = max_pt.x - min_pt.x;
    height = max_pt.y - min_pt.y;
    length = max_pt.z - min_pt.z;
    
    step_width = width;
    
    const Eigen::Vector3f mean_diag = 0.5f*(max_pt.getVector3fMap() + min_pt.getVector3fMap());
	Eigen::Vector3f mean_diag_absolute = stair_dir*mean_diag+ centroid.getVector3fMap().head<3>();
	center = pcl::PointXYZ (mean_diag_absolute[0], mean_diag_absolute[1], mean_diag_absolute[2]);
	
	Eigen::Vector3f initial_point_vector = mean_diag_absolute - length/2*stair_dir.col(2)  - height/2*stair_dir.col(1);
  initial_point = pcl::PointXYZ(initial_point_vector(0),initial_point_vector(1),initial_point_vector(2));
//  translation = -1 * (rotation * initial_point_vector.cast<double>());

//    i2s = ( Eigen::Translation3d (translation) * Eigen::AngleAxisd (rotation));
//	s2i = createAffine3d(stair_dir,initial_point_vector);
//    s2i = i2s.inverse();
    s2i = ( Eigen::Translation3d (initial_point_vector.cast<double>()) * Eigen::AngleAxisd (stair_dir.cast<double>()));
    i2s = s2i.inverse();

	
	float cumulated_height = 0;
	int n_height = 0;
	float cumulated_length = 0;
	int n_length = 0;
	
	for (int Q=0; Q<vLevels.size(); Q++)
	{

		if ((Q == 0) and (vLevels[0].cloud->points.size() > 0))
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr neighbouring_cloud (new pcl::PointCloud<pcl::PointXYZ>);
			if (neighbour_search(vLevels[0].cloud, initial_point, 0.5f, neighbouring_cloud) > 5)
			{				
				vLevels[Q].cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
				*vLevels[Q].cloud = *neighbouring_cloud;
				vLevels[Q].getCentroid();
				vLevels[Q].getMeasurements(stair_dir,neighbouring_cloud);
				vLevels[Q].width = width;
				vLevels[Q].getVertices(stair_dir);
				
				Plane temp_plane;
				temp_plane.vertices.reset(new pcl::PointCloud<pcl::PointXYZ>);
				
				// move the points to the that reference frame
				
				pcl::transformPointCloud(*vLevels[Q].vertices, *temp_plane.vertices, i2s);
				*temp_plane.vertices += *vLevels[Q].vertices;
				vOrientedLevels.push_back(temp_plane);
			}
			else
			{
				Plane temp_plane;
				temp_plane.vertices.reset(new pcl::PointCloud<pcl::PointXYZ>);
				vOrientedLevels.push_back(temp_plane);
			}			
		}
		else if (Q != 0)
		{
			
			vLevels[Q].width = width;
		
			Plane temp_plane;
			temp_plane.vertices.reset(new pcl::PointCloud<pcl::PointXYZ>);
			
			// move the points to the that reference frame
			pcl::transformPointCloud(*vLevels[Q].vertices, *temp_plane.vertices, i2s);
			*temp_plane.vertices += *vLevels[Q].vertices;
			vOrientedLevels.push_back(temp_plane);
		}
		else
		{
			Plane temp_plane;
			temp_plane.vertices.reset(new pcl::PointCloud<pcl::PointXYZ>);
			vOrientedLevels.push_back(temp_plane);
		}
		
		// if (vOrientedLevels[Q].vertices->points.size() > 0)
		// {
		// 	std::cout << Q << std::endl;
		// 	std::cout << vOrientedLevels[Q].vertices->points[0].x << " " << vOrientedLevels[Q].vertices->points[0].y << " " << vOrientedLevels[Q].vertices->points[0].z << std::endl;
		// 	std::cout << vOrientedLevels[Q].vertices->points[1].x << " " << vOrientedLevels[Q].vertices->points[1].y << " " << vOrientedLevels[Q].vertices->points[1].z << std::endl;
		// 	std::cout << vOrientedLevels[Q].vertices->points[2].x << " " << vOrientedLevels[Q].vertices->points[2].y << " " << vOrientedLevels[Q].vertices->points[2].z << std::endl;
		// 	std::cout << vOrientedLevels[Q].vertices->points[3].x << " " << vOrientedLevels[Q].vertices->points[3].y << " " << vOrientedLevels[Q].vertices->points[3].z << std::endl;
		// 	std::cout << vOrientedLevels[Q].vertices->points[4].x << " " << vOrientedLevels[Q].vertices->points[4].y << " " << vOrientedLevels[Q].vertices->points[4].z << std::endl << std::endl;
		// }


		if ((Q>1) or ((Q == 1) and (vOrientedLevels[0].vertices->points.size() > 0)))
		{
			// HEIGHT CORRECTION
			n_height++;
			cumulated_height += vOrientedLevels[Q-1].vertices->points[0].y - vOrientedLevels[Q].vertices->points[0].y;
			
			// LENGTH CORRECTION
			if (vOrientedLevels[Q-1].vertices->points[0].y > vOrientedLevels[Q].vertices->points[0].y) // escaleras de bajada
			{
				vOrientedLevels[Q].vertices->points[2].z = vOrientedLevels[Q-1].vertices->points[0].z;
				vOrientedLevels[Q].vertices->points[3].z = vOrientedLevels[Q-1].vertices->points[0].z;
				vOrientedLevels[Q].vertices->points[4].z = (vOrientedLevels[Q].vertices->points[0].z+vOrientedLevels[Q].vertices->points[2].z)/2;
				
				n_length++;
				cumulated_length += vOrientedLevels[Q].vertices->points[0].z - vOrientedLevels[Q].vertices->points[2].z;
			}
			else // escaleras de subida
			{
				if (Q>1)
				{
					vOrientedLevels[Q-1].vertices->points[0].z = vOrientedLevels[Q].vertices->points[2].z;
					vOrientedLevels[Q-1].vertices->points[1].z = vOrientedLevels[Q].vertices->points[3].z;
					vOrientedLevels[Q-1].vertices->points[4].z = (vOrientedLevels[Q].vertices->points[0].z+vOrientedLevels[Q].vertices->points[2].z)/2;
					
					n_length++;
					cumulated_length += vOrientedLevels[Q-1].vertices->points[0].z - vOrientedLevels[Q-1].vertices->points[2].z;
				}
			}
		}
		// WIDTH CORRECTION
		if (vOrientedLevels[Q].vertices->points.size() > 0)
		{
			vOrientedLevels[Q].vertices->points[0].x = width/2;
			vOrientedLevels[Q].vertices->points[1].x = -width/2;
			vOrientedLevels[Q].vertices->points[2].x = -width/2;
			vOrientedLevels[Q].vertices->points[3].x = width/2;
			vOrientedLevels[Q].vertices->points[4].x = 0;
		}
	}
	
	step_height = fabs(cumulated_height/n_height);
	if (n_length > 0)
	{
		step_length = cumulated_length/n_length;


	
		// Correction of vertices in case of first step and no floor in descending staircase or last step in ascending staircase
		if (vOrientedLevels.size()>2)
		{
			if ((vOrientedLevels[0].vertices->points.size() == 0) and (vOrientedLevels[2].vertices->points[4].y < vOrientedLevels[1].vertices->points[4].y))
			{
				//~ std::cout << "Corrección primer escalón en escaleras de bajada" << std::endl;
				vOrientedLevels[1].vertices->points[2].z = vOrientedLevels[1].vertices->points[0].z-step_length;
				vOrientedLevels[1].vertices->points[3].z = vOrientedLevels[1].vertices->points[0].z-step_length;
				vOrientedLevels[1].vertices->points[4].z = (vOrientedLevels[1].vertices->points[0].z+vOrientedLevels[1].vertices->points[2].z)/2;
			}
			if (vOrientedLevels[2].vertices->points[4].y > vOrientedLevels[1].vertices->points[4].y)
			{
				//~ std::cout << "Corrección último escalón en escaleras de subida " << std::endl;
				vOrientedLevels[vOrientedLevels.size()-1].vertices->points[0].z = vOrientedLevels[vOrientedLevels.size()-1].vertices->points[2].z + step_length;
				vOrientedLevels[vOrientedLevels.size()-1].vertices->points[1].z = vOrientedLevels[vOrientedLevels.size()-1].vertices->points[2].z + step_length;
				vOrientedLevels[vOrientedLevels.size()-1].vertices->points[4].z = (vOrientedLevels[vOrientedLevels.size()-1].vertices->points[0].z+vOrientedLevels[vOrientedLevels.size()-1].vertices->points[2].z)/2;
			}
			
		}
	}

	


	Plane empty_plane;
	vRisers.push_back(empty_plane);
	
	for (int Q = 1; Q< vOrientedLevels.size(); Q++)
	{				
		Plane temp_plane;
		temp_plane.vertices.reset(new pcl::PointCloud<pcl::PointXYZ>);
		if (Q == 1)
		{

			temp_plane.vertices->points.push_back(vOrientedLevels[Q].vertices->points[3]);
			temp_plane.vertices->points.push_back(vOrientedLevels[Q].vertices->points[2]);
			temp_plane.vertices->points.push_back(pcl::PointXYZ(vOrientedLevels[Q].vertices->points[2].x,vOrientedLevels[Q].vertices->points[2].y-step_height,vOrientedLevels[Q].vertices->points[2].z));
			temp_plane.vertices->points.push_back(pcl::PointXYZ(vOrientedLevels[Q].vertices->points[3].x,vOrientedLevels[Q].vertices->points[3].y-step_height,vOrientedLevels[Q].vertices->points[3].z));
			
			temp_plane.vertices->points.push_back(pcl::PointXYZ(0,vOrientedLevels[Q].vertices->points[2].y-step_height/2,vOrientedLevels[Q].vertices->points[2].z));
		}
		else if ((Q>1))// or ((Q == 1) and (vOrientedLevels[0].vertices->points.size() > 0)))
		{
			temp_plane.vertices->points.push_back(vOrientedLevels[Q-1].vertices->points[0]);
			temp_plane.vertices->points.push_back(vOrientedLevels[Q-1].vertices->points[1]);
			temp_plane.vertices->points.push_back(vOrientedLevels[Q].vertices->points[2]);
			temp_plane.vertices->points.push_back(vOrientedLevels[Q].vertices->points[3]);
			temp_plane.vertices->points.push_back(pcl::PointXYZ(0,(vOrientedLevels[Q].vertices->points[0].y+vOrientedLevels[Q-1].vertices->points[0].y)/2,vOrientedLevels[Q].vertices->points[0].z));
		}
		
		vRisers.push_back(temp_plane);

		pcl::transformPointCloud(*vRisers[Q].vertices,*vRisers[Q].vertices,s2i);
	}
	
	for (int Q = 1; Q< vOrientedLevels.size(); Q++)
	{		
		//~ vOrientedLevels[Q].vertices->points[0].y += 0.01f;
		//~ vOrientedLevels[Q].vertices->points[1].y += 0.01f;
		//~ vOrientedLevels[Q].vertices->points[2].y += 0.01f;
		//~ vOrientedLevels[Q].vertices->points[3].y += 0.01f;
		//~ vOrientedLevels[Q].vertices->points[4].y += 0.01f;

		pcl::transformPointCloud(*vOrientedLevels[Q].vertices,*vOrientedLevels[Q].vertices,s2i);
	}
    
    
//     std::cout << "Width :" << width << std::endl;
//     std::cout << "length :" << length << std::endl;
//     std::cout << "height :" << height << std::endl << std::endl;
    
//     std::cout << "Width :" << step_width << std::endl;
//     std::cout << "length :" << step_length << std::endl;
//     std::cout << "height :" << step_height << std::endl << std::endl;
}

int neighbour_search(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointXYZ point, float radius, pcl::PointCloud<pcl::PointXYZ>::Ptr &neighbouring_cloud)
{
  pcl::search::KdTree<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(cloud);
  std::vector<float> pointRadiusSquaredDistance;
  std::vector<int> pointIdxRadiusSearch;
  int neighbours;
  neighbours = kdtree.radiusSearch(point,radius,pointIdxRadiusSearch,pointRadiusSquaredDistance);

  pcl::copyPointCloud(*cloud,pointIdxRadiusSearch,*neighbouring_cloud);

  return neighbours;
}

int neighbour_search(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointXYZ point, float radius)
{
  pcl::search::KdTree<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(cloud);
  std::vector<float> pointRadiusSquaredDistance;
  std::vector<int> pointIdxRadiusSearch;
  int neighbours;
  neighbours = kdtree.radiusSearch(point,radius,pointIdxRadiusSearch,pointRadiusSquaredDistance);

  return neighbours;
}
