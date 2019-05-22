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

#include "RGBD/global_scene.h"

void GlobalScene::findFloor(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
    float z_dist = 2.0; // instead of using the whole cloud, start with the first z_dist meters in direction 'z'
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    // Keep growing z_dist untill a reasonably large enough cloud is found
    const int min_cloud_size = 200;
    while (cloud_filtered->points.size() < min_cloud_size)	{
        pcl::PassThrough<pcl::PointXYZ> pass;
		pass.setInputCloud(cloud);
		pass.setFilterFieldName("z");
		pass.setFilterLimits(0.0, z_dist);
		pass.setFilterLimitsNegative (false);
		pass.filter(*cloud_filtered);
        if (z_dist > 5.0f)
			break;
        else
            z_dist+=0.3f;

	}
	
    // Once we have a filtered cloud, look for up to n_planes_try and evaluate if they met the conditions for valid floor
    int n_planes_try = 10;
	
    while ((!initial_floor) and (n_planes_try >= 0)) {
        if (n_planes_try == 0) {
            z_dist+=0.3f; // If none of the n_planes tried produces a valid initial_floor, increase the z_dist
            if (z_dist > 10.0f)
				break;

			pcl::PassThrough<pcl::PointXYZ> pass;
			pass.setInputCloud(cloud);
			pass.setFilterFieldName("z");
			pass.setFilterLimits(0.0, z_dist);
			pass.filter(*cloud_filtered);

            n_planes_try = 10;
		}
        else {
            initial_floor = this->subtractInitialPlane(cloud_filtered, floor_normal);
            n_planes_try -= 1;
        }
    }
}

bool GlobalScene::subtractInitialPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, Eigen::Vector4f &normal) {
    // We segment using RANSAC for planes with the input cloud, and return the remaining points (not in the plane) in case of further iterations needed.
    // If valid floor is found, the coefficients are returned and the function returns true.

    bool initial_plane = false; // Will change to true if valid floor is found
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>());
	*cloud_filtered=*cloud;    
	
    if (cloud_filtered->points.size()>3){
		// Create the segmentation object for the planar model and set all the parameters
		pcl::SACSegmentation<pcl::PointXYZ> seg;
		pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
		pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
		seg.setOptimizeCoefficients (true);
		seg.setModelType (pcl::SACMODEL_PLANE);
		seg.setMethodType (pcl::SAC_RANSAC);
		seg.setMaxIterations (100);
        seg.setDistanceThreshold (0.04); // aprox. Voxel size
	
		// Segment the largest planar component from the remaining cloud
		seg.setInputCloud (cloud_filtered);
		seg.segment (*inliers, *coefficients);
		
        // Plane coefficients   Ax + By + Cz + D =0
        float A=(coefficients->values[0]);
        float B=(coefficients->values[1]);
        float C=(coefficients->values[2]);
        float D=(coefficients->values[3]);

        if (D < 0) {A = -A; B = -B; C = -C; D = -D;} // To get the plane normal looking towards the origin

        Eigen::Vector3f v_plane(A,B,C);
        Eigen::Vector3f v_floor(0.0f, -sin(float(M_PI_4)), -sin(float(M_PI_4))); // Estimated position of the floor normal considering the camera is looking downwards, circa 45 degrees

        float dot = v_plane.dot(v_floor);
        dot = ( dot < -1.0f ? -1.0f : ( dot > 1.0f ? 1.0f : dot ) ); // to avoid NaNs
        float angle = pcl::rad2deg(acos(dot));

        // Parameters to consider valid floor
        const float angle_threshold = 45; // Valid threshold around estimated floor vector
        const float min_D = 1.0f; // Minimum D value (i.e. minimum distance of the floor to the camera allowed)
        const float max_D = 1.8f; // Maximum D value (i.e. maximum distance of the floor to the camera allowed)

        if (angle < angle_threshold && fabs(D) > min_D && fabs(D) < max_D) {
            std::cout << "Floor found" << std::endl;
            std::cout << "Plane coefficients -> A: " << A << " B: " << B << " C: " << C << " D: " << D << std::endl;
            std::cout << "Angle with respect to the estimated vertical normal = " << angle << std::endl;
            std::cout << "Plane contains " << inliers->indices.size() << " points" << std::endl;

            normal = Eigen::Vector4f(A,B,C,D);

            initial_plane = true;
        }

        // If no initial_plane found, return cloud with remaining points
        if (!initial_plane) {
			pcl::ExtractIndices<pcl::PointXYZ> extract;
			extract.setInputCloud (cloud_filtered);
			extract.setIndices (inliers);
			extract.setNegative (true);
			extract.filter (*cloud);
        }

    }
		
	return (initial_plane);
}

void GlobalScene::computeCamera2FloorMatrix () {
    // Transformation from Camera to Floor reference frame (c2f) is computed knowing one of the axis (the floor normal)
    Eigen::Matrix3f R;
    Eigen::Vector3f t = Eigen::Vector3f::Zero();

    float a,b,c,d;
    c = sqrt(1/(1+floor_normal(2)*floor_normal(2)/(floor_normal(1)*floor_normal(1))));
    d = -c*floor_normal(2)/floor_normal(1);
    b = 1/(floor_normal(0)+(c*floor_normal(1)*(c*floor_normal(1)-d*floor_normal(2)))/floor_normal(0)-(d*floor_normal(2)*(c*floor_normal(1)-d*floor_normal(2)))/floor_normal(0));
    a = b*(c*floor_normal(1)-d*floor_normal(2))/floor_normal(0);
        
    R << a, -b*c, b*d, b, a*c, -a*d, 0, d, c;
    t(1) = -a*floor_normal(3);

    c2f = ( Eigen::Translation3d (t.cast<double>()) * Eigen::AngleAxisd (R.cast<double>()));
    f2c = c2f.inverse();
    person_height = float(fabs(c2f(1,3)));
}


void GlobalScene::computeCamera2AbsoluteMatrix(Eigen::Affine3d f2a) {
  c2a = f2a*c2f;
  a2c = c2a.inverse();
}

void GlobalScene::computeCamera2AbsoluteMatrixWithOdometry(Eigen::Affine3d c2c0) {
  c2a = c02a*c2c0;
  a2c = c2a.inverse();
}

void GlobalScene::computeIncrementalCamera2FloorMatrix () {
	Eigen::Affine3d old_c2f = c2f;
	
    Eigen::Matrix3f R;
    Eigen::Vector3f t = Eigen::Vector3f (0.f, 0.f, 0.f);
    
    float a,b,c,d;
    
    c = sqrt(1/(1+floor_normal(2)*floor_normal(2)/(floor_normal(1)*floor_normal(1))));
    d = -c*floor_normal(2)/floor_normal(1);
    b = 1/(floor_normal(0)+(c*floor_normal(1)*(c*floor_normal(1)-d*floor_normal(2)))/floor_normal(0)-(d*floor_normal(2)*(c*floor_normal(1)-d*floor_normal(2)))/floor_normal(0));
    a = b*(c*floor_normal(1)-d*floor_normal(2))/floor_normal(0);
        
    R << a, -b*c, b*d, b, a*c, -a*d, 0, d, c;
	t(1) = person_height;

    c2f = ( Eigen::Translation3d (t.cast<double>()) * Eigen::AngleAxisd (R.cast<double>()));
    f2c = c2f.inverse();
    
    
    Eigen::Matrix3d rot1,rot2,rot3;
    rot1 = old_c2f.rotation();
    rot2 = f2c.rotation();
    rot3 = rot1*rot2;
    f2f = Eigen::Translation3d(Eigen::Vector3d(0,0,0)) * Eigen::AngleAxisd(rot3);

}



void GlobalScene::findFloorFast(std::vector<Plane> vPlanes) {
    new_floor = false;
    for (size_t Q=0; Q<vPlanes.size();Q++)	{

        float dot = vPlanes[Q].coeffs.head<3>().dot(floor_normal.head<3>());
        dot = ( dot < -1.0f ? -1.0f : ( dot > 1.0f ? 1.0f : dot ) );

        // Threshold to consider new valid floor
        const float angle_threshold = 15; // degrees with respect to previous floor_normal
        const float D_threshold = 0.08f; // Distance to camera with respect to previous D

        if ((pcl::rad2deg(acos(dot)) < angle_threshold) and (fabs(fabs(vPlanes[Q].coeffs[3])-fabs(floor_normal[3]))<=D_threshold)) {
            floor_normal = vPlanes[Q].coeffs;
            new_floor = true;
            current_floor = int(Q);
            break;
		}
	}
}

void GlobalScene::updateManhattanDirections(Eigen::Matrix3f manhattan_dirs) {
    if (!has_manhattan) {  // If manhattan has never been found, set eigDx
        eigDx = manhattan_dirs;
        has_manhattan = true;
    }
    else {

        // Get the angle (as a turn in Y) of Z axis (in floor reference) from old to new reference
        double angle_z = double(acos(manhattan_dirs.col(2).dot(eigDx.col(2))));
        Eigen::Vector3f cross_z = manhattan_dirs.col(2).cross(eigDx.col(2));
        if (eigDx.col(1).dot(cross_z) < 0) { // Or > 0
          angle_z = -angle_z;
        }


        // Keep the axis equally oriented (i.e. Z and X pointing at the same points) even if you move around
        // Figure in which quadrant the angle_z is, and change eigDx in consonance
        if (angle_z >= M_PI_4 && angle_z < 3*M_PI_4)
        {
            eigDx.col(0) = -manhattan_dirs.col(2);
            eigDx.col(1) = manhattan_dirs.col(1);
            eigDx.col(2) = manhattan_dirs.col(0);
        }
        else if (angle_z >= 3*M_PI_4 || angle_z <= -3*M_PI_4)
        {
            eigDx.col(0) = -manhattan_dirs.col(0);
            eigDx.col(1) = manhattan_dirs.col(1);
            eigDx.col(2) = -manhattan_dirs.col(2);
        }
        else if (angle_z > -3*M_PI_4 && angle_z < -M_PI_4)
        {
            eigDx.col(0) = manhattan_dirs.col(2);
            eigDx.col(1) = manhattan_dirs.col(1);
            eigDx.col(2) = -manhattan_dirs.col(0);
        }
        else
        {
            eigDx.col(0) = manhattan_dirs.col(0);
            eigDx.col(1) = manhattan_dirs.col(1);
            eigDx.col(2) = manhattan_dirs.col(2);

        }

    }
}

void GlobalScene::updateFloorWithManhattan() {
    floor_normal.head<3>() = eigDx.col(1);
    c2f.matrix().block<3,3>(0,0) = eigDx.cast<double>().transpose();
    f2c = c2f.inverse();
}


void GlobalScene::getManhattanDirections(CurrentScene &scene) {

    if (new_floor) {  // In findFloorFast has found a new floor
        scene.getManhattanDirectionsFromPlanesWithFloor(f2c);
    }

    if (scene.has_manhattan == false) { // if previous function did not work
        scene.getManhattanDirectionsFromPlanes();

        if (scene.has_manhattan == false)
            scene.getManhattanDirectionsFromNormals();
    }

    if (scene.has_manhattan)  {
        updateManhattanDirections(scene.eigDx);
        updateFloorWithManhattan();
    }
}
