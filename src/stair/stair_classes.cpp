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

Plane Stair::getBestStep(){
  // We chose the "best step" as the one that maximizes the extent
  // (i.e. ratio between contour area and bounding rectangle) in all steps at the same time
  // Thus, we compute the main directions of the step (main_dir) and sum the rectangle area of all steps following that direction

  size_t index = 0;
  float max_extent = 0;
  float sum_of_contour_areas = 0;

  for (size_t Q = 1; Q<vLevels.size(); Q++)
      sum_of_contour_areas += vLevels[Q].getContourArea();

  for (size_t Q = 1; Q<vLevels.size(); Q++) {
    if (vLevels[Q].main_dir.isZero(0))
      vLevels[Q].getPrincipalDirections();

    getStairDirFromPlane(vLevels[Q]);

    float sum_of_rectangle_areas = 0;
    for (size_t P = 1; P<vLevels.size(); P++)
      sum_of_rectangle_areas += vLevels[P].getRectangleArea(vLevels[Q].main_dir);

    float extent = sum_of_contour_areas/sum_of_rectangle_areas;

    if (extent > max_extent){
      max_extent = extent;
      index = Q;
    }
  }
  return vLevels[index];
}


void Stair::getStairDirFromPlane(Plane &plane) {
  // The stair dir is with Z pointing towards the stair, Y pointing upwards and X to the left
  // Knowing that the principal directions are, by columns:
  // First, the one with smaller eigenvalue (corresponding to height --> Y)
  // Second the one with intermediate eigenvalue (corresponding to length --> Z)
  // Third, the one with larger eigenvalue (corresponding to width --> X)

  Eigen::Vector3f dx = plane.main_dir.col(2);
  Eigen::Vector3f dy = plane.main_dir.col(0);
  Eigen::Vector3f dz = plane.main_dir.col(1);

  // Check that Y go upwards, and Z frontwards
  if (dy(1) > 0) // In normal position of the camera, Y points downwards
      dy = -dy;
  if (acos(plane.centroid.getVector3fMap().normalized().dot(dz)) > float(M_PI_2)) // Z must be directed towards the staircase
      dz = -dz;
  dx = dy.cross(dz);

  plane.main_dir << dx, dy, dz;

}

void Stair::getInitialStairVolume(Eigen::Matrix3f manhattan_dirs, Eigen::Matrix3f standalone_dirs, bool has_manhattan) {
    // We have two options to obtain the model of the staircase:
    // First, with the Manhattan dirs (hopefully coming from previous processing)
    // Second, standalone method (coming from principal directions of the "best step")
    // We compute the cubic volume that encapsulates all points in the levels for each method
    // The method that produces smaller volume should fit the staircase better.

    pcl::PointCloud<pcl::PointXYZ>::Ptr allPoints (new pcl::PointCloud<pcl::PointXYZ>);
    for (size_t Q=1; Q<vLevels.size(); Q++)
        if (vLevels[Q].contour->points.size() > 0)
            *allPoints += *vLevels[Q].contour; // Less points, use if possible
        else
            *allPoints += *vLevels[Q].cloud;

	Eigen::Vector4f vector_centroid;
	pcl::compute3DCentroid(*allPoints,vector_centroid);
	
    // 1. PCA Method
    // Transform points to a reference system centered in the centroid of the volume
    Eigen::Matrix3d rotation_standalone = standalone_dirs.transpose().cast<double>();
    Eigen::Vector3d translation_standalone = -1 * (rotation_standalone * vector_centroid.head<3>().cast<double>());
    Eigen::Affine3d i2s_standalone = ( Eigen::Translation3d (translation_standalone) * Eigen::AngleAxisd (rotation_standalone));
    pcl::PointCloud<pcl::PointXYZ> cPoints_standalone;
    pcl::transformPointCloud(*allPoints, cPoints_standalone, i2s_standalone);
	
    // Get min and max points in the volume
    pcl::PointXYZ min_pt, max_pt;
    pcl::PointXYZ min_pt_standalone, max_pt_standalone;
    pcl::getMinMax3D(cPoints_standalone, min_pt_standalone, max_pt_standalone);
    
    // Get measurements and volume
    float width_standalone = fabs(max_pt_standalone.x - min_pt_standalone.x);
    float height_standalone = fabs(max_pt_standalone.y - min_pt_standalone.y);
    float length_standalone = fabs(max_pt_standalone.z - min_pt_standalone.z);   
    float volume_standalone = width_standalone*height_standalone*length_standalone;

    float width, length, height;

    if (has_manhattan) {   // If scene has Manhattan
        // 2. MANHATTAN Method
        // First, check if axes are well directed towards the staircase. Use best step dirs for reference (it is always well oriented towards the staircase)
        // This is necessary since we want the axis X and Z to follow our established convention for the staircase, and may not match the axis from Manhattan

        if (acos(fabs(manhattan_dirs.col(2).dot(standalone_dirs.col(2)))) > acos(fabs(manhattan_dirs.col(0).dot(standalone_dirs.col(2))))){ // Z of Manhattan is closer to X of standalone
            Eigen::Matrix3f aux = manhattan_dirs;
            Eigen::Vector3f cross_z = standalone_dirs.col(2).cross(manhattan_dirs.col(2));
            if (manhattan_dirs.col(1).dot(cross_z) > 0) { // turn right
                manhattan_dirs.col(0) = aux.col(2);
                manhattan_dirs.col(2) = -aux.col(0);
            }
            else { // turn left
                manhattan_dirs.col(0) = -aux.col(2);
                manhattan_dirs.col(2) = aux.col(0);
            }
        }
        else if (acos((manhattan_dirs.col(2).dot(standalone_dirs.col(2)))) > acos(((-manhattan_dirs.col(2)).dot(standalone_dirs.col(2))))) { // Backwards orientation:
            manhattan_dirs.col(0) = -manhattan_dirs.col(0);
            manhattan_dirs.col(2) = -manhattan_dirs.col(2);
        }

        // Repeat as in 1.
		Eigen::Matrix3d rotation_manhattan = manhattan_dirs.transpose().cast<double>();
        Eigen::Vector3d translation_manhattan = -1 * (rotation_manhattan * vector_centroid.head<3>().cast<double>());
		Eigen::Affine3d i2s_manhattan = ( Eigen::Translation3d (translation_manhattan) * Eigen::AngleAxisd (rotation_manhattan));
		pcl::PointCloud<pcl::PointXYZ> cPoints_manhattan;
	    pcl::transformPointCloud(*allPoints, cPoints_manhattan, i2s_manhattan);

	    pcl::PointXYZ min_pt_manhattan, max_pt_manhattan;
	    pcl::getMinMax3D(cPoints_manhattan, min_pt_manhattan, max_pt_manhattan);

	    float width_manhattan = fabs(max_pt_manhattan.x - min_pt_manhattan.x);
	    float height_manhattan = fabs(max_pt_manhattan.y - min_pt_manhattan.y);
	    float length_manhattan = fabs(max_pt_manhattan.z - min_pt_manhattan.z);
        float volume_manhattan = width_manhattan*height_manhattan*length_manhattan;

        // Our default will be Manhattan unless standalone is smaller (optional threshold)
        const float volume_percentage_threshold = 1.00f;

        if (volume_standalone < volume_percentage_threshold*volume_manhattan) {
            stair_dir = standalone_dirs;

            width = width_standalone;
            length = length_standalone;
            height = height_standalone;

            max_pt = max_pt_standalone;
            min_pt = min_pt_standalone;
        }
        else {
            stair_dir = manhattan_dirs;

            width = width_manhattan;
            length = length_manhattan;
            height = height_manhattan;

            max_pt = max_pt_manhattan;
            min_pt = min_pt_manhattan;
        }
	}
    else {
        stair_dir = standalone_dirs;

		width = width_standalone;
    	length = length_standalone;
    	height = height_standalone;

    	max_pt = max_pt_standalone;
    	min_pt = min_pt_standalone;
	}

    step_width = width; // The width of the steps will be the width of the volume
    
    // Compute center of the volume (not centroid but the center of the cuboid)
    const Eigen::Vector3f mean_diag = 0.5f*(max_pt.getVector3fMap() + min_pt.getVector3fMap());
    Eigen::Vector3f mean_diag_absolute = stair_dir*mean_diag + vector_centroid.head<3>();

    // Compute the initial point of the staircase: will be the center of the edge of the first step.
    // Since we have not included floor (level 0), it is directly the edge of the parallelogram.
	Eigen::Vector3f initial_point_vector;
	if (type == "down")
		initial_point_vector = mean_diag_absolute - length/2*stair_dir.col(2)  + height/2*stair_dir.col(1);
	else
        initial_point_vector = mean_diag_absolute - length/2*stair_dir.col(2)  - height/2*stair_dir.col(1);

    initial_point = pcl::PointXYZ(initial_point_vector(0),initial_point_vector(1),initial_point_vector(2));

    // Compute the transformations (staircase to camera at frame i and the inverse)
    s2i = ( Eigen::Translation3d (initial_point_vector.cast<double>()) * Eigen::AngleAxisd (stair_dir.cast<double>()));
    i2s = s2i.inverse();

}

void Stair::getInitialStepVertices(){
    this->getInitialStepVertices(stair_dir);
}


void Stair::getInitialStepVertices(Eigen::Matrix3f & dir) {
    Eigen::Matrix3f aux = dir;

    for (size_t Q=1; Q<vLevels.size(); Q++) {
        vLevels[Q].getMeasurements(dir);
        vLevels[Q].getVertices(dir);
    }
}


void Stair::getExactStepVertices() {
    // In this function we obtain the final measurements of the staircase, as well as the transformation to the stair reference frame
    // We iterate through the steps, computing the vertices of the vLevels (in vOrientedLevels)

    // If there is floor, compute the neighbouring cloud and its vertices for further iterations
    if (vLevels[0].cloud->points.size() > 0) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr neighbouring_cloud (new pcl::PointCloud<pcl::PointXYZ>);
        // Check if there are points (at least 5, for example) in around half a meter from initial point of the stair
        if (neighbour_search(vLevels[0].cloud, initial_point, 0.5f, neighbouring_cloud) > 5) {
            vLevels[0].cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
            *vLevels[0].cloud = *neighbouring_cloud; // Instead of using the whole cloud, use just the neighbouring cloud
            vLevels[0].getCentroid();
            vLevels[0].getMeasurements(stair_dir,neighbouring_cloud);
            vLevels[0].width = step_width;
            vLevels[0].getVertices(stair_dir);

            // Add level 0 to oriented levels (in staircase coordinates)
            Plane temp_plane;
            temp_plane.vertices.reset(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::transformPointCloud(*vLevels[0].vertices, *temp_plane.vertices, i2s);
            vOrientedLevels.push_back(temp_plane);
        }
    }


    if (vOrientedLevels.size() == 0) {// There is no floor or it is not connected, then just add empty plane to oriented levels
        Plane temp_plane;
        temp_plane.vertices.reset(new pcl::PointCloud<pcl::PointXYZ>);
        vOrientedLevels.push_back(temp_plane);
        vLevels[0].vertices.reset(new pcl::PointCloud<pcl::PointXYZ>);

    }

    // Initial point/s2i slight correction.
    // The iniitial point should be:
    // - At the centroid of the first step in Y
    // - At 0 in X
    // - In Z:
    // 1. For downstairs, at the edge of the level 0 step (the floor) in Z
    // 2. For upstairs, at 0 (i.e. the edge of the level 1 step)
    if ((type == "down") and (vOrientedLevels[0].vertices->points.size() > 0)) {
        Eigen::Matrix3d rotation_i2s = i2s.rotation();
        Eigen::Vector3f first_step_centroid2s;
        pcl::transformPoint(vLevels[1].centroid.getVector3fMap(),first_step_centroid2s,i2s.cast<float>());
        Eigen::Vector3f initial_vector_point_aux = Eigen::Vector3f(0,first_step_centroid2s(1),vOrientedLevels[0].vertices->points[0].z);
        pcl::transformPoint(initial_vector_point_aux,initial_vector_point_aux, s2i.cast<float>());
        Eigen::Vector3d translation = -1 * (rotation_i2s * initial_vector_point_aux.cast<double>());

        initial_point = pcl::PointXYZ(initial_vector_point_aux(0),initial_vector_point_aux(1),initial_vector_point_aux(2));

        i2s = ( Eigen::Translation3d (translation) * Eigen::AngleAxisd (rotation_i2s));
        s2i = i2s.inverse();

        // Correct vOrientedLevels at zero
        pcl::transformPointCloud(*vLevels[0].vertices, *vOrientedLevels[0].vertices, i2s);

    }
    else if (type == "up") {
        Eigen::Matrix3d rotation_i2s = i2s.rotation();
        Eigen::Vector3f first_step_centroid2s;
        pcl::transformPoint(vLevels[1].centroid.getVector3fMap(),first_step_centroid2s,i2s.cast<float>());
        Eigen::Vector3f initial_vector_point_aux = Eigen::Vector3f(0,first_step_centroid2s(1),0);
        pcl::transformPoint(initial_vector_point_aux,initial_vector_point_aux, s2i.cast<float>());
        Eigen::Vector3d translation = -1 * (rotation_i2s * initial_vector_point_aux.cast<double>());

        initial_point = pcl::PointXYZ(initial_vector_point_aux(0),initial_vector_point_aux(1),initial_vector_point_aux(2));

        i2s = ( Eigen::Translation3d (translation) * Eigen::AngleAxisd (rotation_i2s));
        s2i = i2s.inverse();

        // Correct vOrientedLevels at zero
        if (vLevels[0].cloud->points.size() > 0)
            pcl::transformPointCloud(*vLevels[0].vertices, *vOrientedLevels[0].vertices, i2s);
    }

    // The final length and height will be averaged, thus we need to cumulate and count steps
	float cumulated_height = 0;
	int n_height = 0;
	float cumulated_length = 0;
	int n_length = 0;
	
    for (size_t Q=1; Q<vLevels.size(); Q++)	{

        // Add vertices to vOrientedLevels
        Plane temp_plane;
        temp_plane.vertices.reset(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*vLevels[Q].vertices, *temp_plane.vertices, i2s);
        vOrientedLevels.push_back(temp_plane);
		

        // Corrections of height and length
        if ((Q>1) or ((Q == 1) and (vOrientedLevels[0].vertices->points.size() > 0))){
			// HEIGHT CORRECTION
			n_height++;
			cumulated_height += vOrientedLevels[Q-1].vertices->points[0].y - vOrientedLevels[Q].vertices->points[0].y;
			
			// LENGTH CORRECTION
            if (type == "down") {
                // Set vertices 2 and 3 of current level Q to be at the edge of the step above Q-1
				vOrientedLevels[Q].vertices->points[2].z = vOrientedLevels[Q-1].vertices->points[0].z;
				vOrientedLevels[Q].vertices->points[3].z = vOrientedLevels[Q-1].vertices->points[0].z;
				vOrientedLevels[Q].vertices->points[4].z = (vOrientedLevels[Q].vertices->points[0].z+vOrientedLevels[Q].vertices->points[2].z)/2;
				
                // Compute current step length after edge correction
                float current_step_length = vOrientedLevels[Q].vertices->points[0].z - vOrientedLevels[Q].vertices->points[2].z;
                n_length++;
                cumulated_length += current_step_length;
			}
            else {
                if (Q>1){
                    // Set vertices 0 and 1 of previous level Q-1 to be at the edge of the step below Q
					vOrientedLevels[Q-1].vertices->points[0].z = vOrientedLevels[Q].vertices->points[2].z;
					vOrientedLevels[Q-1].vertices->points[1].z = vOrientedLevels[Q].vertices->points[3].z;
					vOrientedLevels[Q-1].vertices->points[4].z = (vOrientedLevels[Q].vertices->points[0].z+vOrientedLevels[Q].vertices->points[2].z)/2;
					
                    // Compute current step length (of previous step) after edge correction
                    float current_step_length = vOrientedLevels[Q-1].vertices->points[0].z - vOrientedLevels[Q-1].vertices->points[2].z;
                    n_length++;
                    cumulated_length += current_step_length;
				}
			}
		}

		// WIDTH CORRECTION
        if (vOrientedLevels[Q].vertices->points.size() > 0)	{
            vOrientedLevels[Q].vertices->points[0].x = step_width/2;
            vOrientedLevels[Q].vertices->points[1].x = -step_width/2;
            vOrientedLevels[Q].vertices->points[2].x = -step_width/2;
            vOrientedLevels[Q].vertices->points[3].x = step_width/2;
			vOrientedLevels[Q].vertices->points[4].x = 0;
		}
	}
	
    // Compute final step height and length
    if (n_height > 0) {
        step_height = fabs(cumulated_height/n_height);

    }
    if (n_length > 0) {
        step_length = cumulated_length/n_length;

        // If step length is known (more than two levels), correction of vertices and initial point in case of first step and no floor and last step in ascending staircase
        if (vOrientedLevels.size()>2) {
            if (type == "down"){
                if (vOrientedLevels[0].vertices->points.size() == 0){
                    vOrientedLevels[1].vertices->points[2].z = vOrientedLevels[1].vertices->points[0].z-step_length;
                    vOrientedLevels[1].vertices->points[3].z = vOrientedLevels[1].vertices->points[0].z-step_length;
                    vOrientedLevels[1].vertices->points[4].z = (vOrientedLevels[1].vertices->points[0].z+vOrientedLevels[1].vertices->points[2].z)/2;

                    // It is necessary to move the initial point as well:
                    Eigen::Vector3f initial_vector_point = Eigen::Vector3f(0,vOrientedLevels[1].vertices->points[0].y,vOrientedLevels[1].vertices->points[0].z-step_length);
                    pcl::transformPoint(initial_vector_point,initial_vector_point, s2i.cast<float>());
                    initial_point = pcl::PointXYZ(initial_vector_point(0),initial_vector_point(1),initial_vector_point(2));
                    s2i.translation() = Eigen::Vector3d(double(initial_point.x), double(initial_point.y), double(initial_point.z));
                    i2s = s2i.inverse();
                }
            }
            else if (type == "up"){
                // Last step correction
                vOrientedLevels[vOrientedLevels.size()-1].vertices->points[0].z = vOrientedLevels[vOrientedLevels.size()-1].vertices->points[2].z + step_length;
                vOrientedLevels[vOrientedLevels.size()-1].vertices->points[1].z = vOrientedLevels[vOrientedLevels.size()-1].vertices->points[2].z + step_length;
                vOrientedLevels[vOrientedLevels.size()-1].vertices->points[4].z = (vOrientedLevels[vOrientedLevels.size()-1].vertices->points[0].z+vOrientedLevels[vOrientedLevels.size()-1].vertices->points[2].z)/2;

                if (vOrientedLevels[0].vertices->points.size() == 0) {
                    // It is necessary to move the initial point as well:
                    Eigen::Vector3f initial_vector_point = Eigen::Vector3f(0,vOrientedLevels[1].vertices->points[0].y,vOrientedLevels[1].vertices->points[0].z-step_length);
                    pcl::transformPoint(initial_vector_point,initial_vector_point, s2i.cast<float>());
                    initial_point = pcl::PointXYZ(initial_vector_point(0),initial_vector_point(1),initial_vector_point(2));
                    s2i.translation() = Eigen::Vector3d(double(initial_point.x), double(initial_point.y), double(initial_point.z));
                    i2s = s2i.inverse();
                }
            }
        }
    }
    else if (vOrientedLevels.size() == 2){ // If only one step, leave step_length as length of the step
        step_length = vOrientedLevels[1].vertices->points[0].z - vOrientedLevels[1].vertices->points[2].z;
    }

	

//    // Add risers, if necessary
//    Plane empty_plane;
//    vRisers.push_back(empty_plane);
	
//    for (size_t Q = 1; Q< vOrientedLevels.size(); Q++) {
//        Plane temp_plane;
//        temp_plane.vertices.reset(new pcl::PointCloud<pcl::PointXYZ>);
//        if (Q == 1) {
//            temp_plane.vertices->points.push_back(vOrientedLevels[Q].vertices->points[3]);
//            temp_plane.vertices->points.push_back(vOrientedLevels[Q].vertices->points[2]);
//            temp_plane.vertices->points.push_back(pcl::PointXYZ(vOrientedLevels[Q].vertices->points[2].x,vOrientedLevels[Q].vertices->points[2].y-step_height,vOrientedLevels[Q].vertices->points[2].z));
//            temp_plane.vertices->points.push_back(pcl::PointXYZ(vOrientedLevels[Q].vertices->points[3].x,vOrientedLevels[Q].vertices->points[3].y-step_height,vOrientedLevels[Q].vertices->points[3].z));
//            temp_plane.vertices->points.push_back(pcl::PointXYZ(0,vOrientedLevels[Q].vertices->points[2].y-step_height/2,vOrientedLevels[Q].vertices->points[2].z));
//        }
//        else if ((Q>1)) {// or ((Q == 1) and (vOrientedLevels[0].vertices->points.size() > 0)))
//            temp_plane.vertices->points.push_back(vOrientedLevels[Q-1].vertices->points[0]);
//            temp_plane.vertices->points.push_back(vOrientedLevels[Q-1].vertices->points[1]);
//            temp_plane.vertices->points.push_back(vOrientedLevels[Q].vertices->points[2]);
//            temp_plane.vertices->points.push_back(vOrientedLevels[Q].vertices->points[3]);
//            temp_plane.vertices->points.push_back(pcl::PointXYZ(0,(vOrientedLevels[Q].vertices->points[0].y+vOrientedLevels[Q-1].vertices->points[0].y)/2,vOrientedLevels[Q].vertices->points[0].z));
//        }
//        vRisers.push_back(temp_plane);

//        pcl::transformPointCloud(*vRisers[Q].vertices,*vRisers[Q].vertices,s2i);
//    }
	
    // Set oriented levels in camera coordinates again.
    for (size_t Q = 1; Q< vOrientedLevels.size(); Q++)
		pcl::transformPointCloud(*vOrientedLevels[Q].vertices,*vOrientedLevels[Q].vertices,s2i);

    
}

bool Stair::validateStaircase() {
    // The dimensions of the staircase are compared to the measurements from the regulations:

    bool valid = true;

    if (step_length < k_length_min) {
        std::cout << "INVALID STAIRCASE, LENGTH (" <<  step_length*100 << "cm) IS SMALLER THAN MINIMUM" << std::endl;
        return false;
    }

    if (step_height < k_height_min || step_height > k_height_max) {
        std::cout << "INVALID STAIRCASE, HEIGHT (" <<  step_height*100 << "cm) IS NOT VALID" << std::endl;
        return false;
    }

    float sum = 2*step_height + step_length;

    if (sum < k_sum_min || sum > k_sum_max) {
        std::cout << "INVALID STAIRCASE, SUM OF LENGTH AND HEIGHT IS NOT VALID" << std::endl;
        return false;
    }

    return valid;
}

void Stair::modelStaircase(Eigen::Matrix3f main_dir, bool has_manhattan){
    Plane best_step = this->getBestStep();
    this->getInitialStairVolume(main_dir, best_step.main_dir, has_manhattan);
    this->getInitialStepVertices();
    this->getExactStepVertices();
}


int neighbour_search(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointXYZ point, float radius, pcl::PointCloud<pcl::PointXYZ>::Ptr &neighbouring_cloud) {
  pcl::search::KdTree<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(cloud);
  std::vector<float> pointRadiusSquaredDistance;
  std::vector<int> pointIdxRadiusSearch;
  int neighbours;
  neighbours = kdtree.radiusSearch(point,radius,pointIdxRadiusSearch,pointRadiusSquaredDistance);

  pcl::copyPointCloud(*cloud,pointIdxRadiusSearch,*neighbouring_cloud);

  return neighbours;
}

int neighbour_search(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointXYZ point, float radius) {
  pcl::search::KdTree<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(cloud);
  std::vector<float> pointRadiusSquaredDistance;
  std::vector<int> pointIdxRadiusSearch;
  int neighbours;
  neighbours = kdtree.radiusSearch(point,radius,pointIdxRadiusSearch,pointRadiusSquaredDistance);

  return neighbours;
}
