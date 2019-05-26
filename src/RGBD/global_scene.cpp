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


void GlobalScene::reset(){
    c2f.setIdentity();
    f2c.setIdentity();
    main_dir.setIdentity();

    initial_floor_ = false;
    new_floor_ = false;
    has_manhattan_ = false;

    floor_normal_ = Eigen::Vector4f::Zero();
    person_height_ = 0;

}

void GlobalScene::findFloor(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {
    // To find the floor, instead of using the whole cloud,
    // it starts with the first z_dist meters in direction 'z'.
    // With this, the floor detected is always close to the person.
    float z_dist = 1.0;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    // Keep growing z_dist untill a reasonably large enough cloud is found
    const int k_min_cluster_size = 500;
    const float k_max_z_dist = 5.0;
    const float k_z_dist_increase = 0.1f;

    while (cloud_filtered->points.size() < k_min_cluster_size)	{
        pcl::PassThrough<pcl::PointXYZ> pass;
		pass.setInputCloud(cloud);
		pass.setFilterFieldName("z");
		pass.setFilterLimits(0.0, z_dist);
		pass.setFilterLimitsNegative (false);
		pass.filter(*cloud_filtered);
        if (z_dist > k_max_z_dist)
            break;
        else
            z_dist += k_z_dist_increase;

	}
	
    // Once we have a filtered cloud, look for up to n_planes_try and evaluate if they met the conditions for valid floor
    const int k_n_planes_try = 5;
    int n_planes_try = k_n_planes_try;
	
    while ((!initial_floor_) and (n_planes_try >= 0)) {
        if (n_planes_try == 0) {
            z_dist += k_z_dist_increase; // If none of the n_planes tried produces a valid initial_floor, increase the z_dist
            if (z_dist > k_max_z_dist)
				break;

			pcl::PassThrough<pcl::PointXYZ> pass;
			pass.setInputCloud(cloud);
			pass.setFilterFieldName("z");
			pass.setFilterLimits(0.0, z_dist);
			pass.filter(*cloud_filtered);

            n_planes_try = k_n_planes_try;
		}
        else {
            initial_floor_ = this->subtractInitialPlane(cloud_filtered, floor_normal_);
            n_planes_try -= 1;
        }
    }
}

bool GlobalScene::subtractInitialPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, Eigen::Vector4f &normal) {
    // We segment using RANSAC for planes with the input cloud, and return the remaining points (not in the plane) in case of further iterations needed.
    // If valid floor is found, the coefficients are returned and the function returns true.

    bool initial_plane = false; // Will change to true if valid floor is found

    const int k_min_plane_size = 300;
    const float k_min_percentage_size = 0.30f;
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>());
	*cloud_filtered=*cloud;    
	
    if (cloud_filtered->points.size()>k_min_plane_size){ // Three points at least to find a plane
        // Perform RANSAC segmentation
        pcl::SACSegmentation<pcl::PointXYZ> seg;
		pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
		pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
		seg.setOptimizeCoefficients (true);
		seg.setModelType (pcl::SACMODEL_PLANE);
		seg.setMethodType (pcl::SAC_RANSAC);
        seg.setMaxIterations (100);
        seg.setDistanceThreshold (0.02); // aprox. Voxel size / 2
	
        // Segment the largest planar component from the cloud
		seg.setInputCloud (cloud_filtered);
        seg.segment (*inliers, *coefficients);

        // Either the plane is very large (at least k_min_plane_size points) or less size but occupying large part of the scene (high percentage of the initial cloud)
        if (inliers->indices.size() > k_min_plane_size ||
                (inliers->indices.size() > float(k_min_plane_size)*k_min_percentage_size && float(inliers->indices.size())/float(cloud_filtered->points.size()) > k_min_percentage_size)) {

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
            const float k_angle_threshold = 30; // Valid threshold around estimated floor vector
            const float k_min_D = 1.0f; // Minimum D value (i.e. minimum distance of the floor to the camera allowed)
            const float k_max_D = 1.8f; // Maximum D value (i.e. maximum distance of the floor to the camera allowed)

            if (angle < k_angle_threshold && fabs(D) > k_min_D && fabs(D) < k_max_D) {
//                std::cout << std::endl <<"-- Floor found --" << std::endl;
//                std::cout << "Plane coefficients -> A: " << A << " B: " << B << " C: " << C << " D: " << D << std::endl;
//                std::cout << "Angle with respect to the estimated vertical normal = " << angle << std::endl;
//                std::cout << "Plane contains " << inliers->indices.size() << " points" << std::endl << std::endl;

                normal = Eigen::Vector4f(A,B,C,D);

                initial_plane = true;
            }
        }
        else {
            initial_plane = false;
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

void GlobalScene::computeCamera2FloorMatrix (Eigen::Vector4f floor_normal) {
    // Transformation from Camera to Floor reference frame (c2f) is computed knowing one of the axis (the floor normal, corresponding to Y in our convention)
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
    person_height_ = float(fabs(c2f(1,3)));
}

void GlobalScene::findFloorFast(std::vector<Plane> vPlanes) {
    // Instead of performing a RANSAC each time, we can use the planes from region growing (computed in current scene)

    // Threshold to consider vPlanes[Q] a new valid floor
    const float k_angle_threshold = 15; // degrees with respect to previous floor_normal
    const float k_D_threshold = 0.08f; // Distance to camera with respect to previous D

    new_floor_ = false;
    for (size_t Q=0; Q<vPlanes.size();Q++)	{

        float dot = vPlanes[Q].coeffs.head<3>().dot(floor_normal_.head<3>());
        dot = ( dot < -1.0f ? -1.0f : ( dot > 1.0f ? 1.0f : dot ) );
        float angle = pcl::rad2deg(acos(dot));

        if ((angle < k_angle_threshold) and (fabs(fabs(vPlanes[Q].coeffs[3])-fabs(floor_normal_[3]))<=k_D_threshold)) {
            floor_normal_ = vPlanes[Q].coeffs;
            new_floor_ = true;
            break;
		}
	}
}

void GlobalScene::getManhattanDirections(CurrentScene &scene) {
    // Given current scene, compute Manhattan

    if (new_floor_ || initial_floor_) {  // In findFloorFast has found a new floor, or has been previously computed (and assumed to be more or less constant), perform computation assuming known Y
        scene.getManhattanDirectionsFromPlanesWithFloor(f2c); // Using planes

        if (scene.has_manhattan_ == false) {
            scene.getManhattanDirectionsFromNormalsWithFloor(f2c,c2f); // If that didn't work, use normals
        }
    }
    else { // This functions compute a new Manhattan direction from scratch (without knowing floor), including thus floor_normal_
        if (scene.has_manhattan_ == false) {
            scene.getManhattanDirectionsFromPlanes();

            if (scene.has_manhattan_ == false){ // If still not enough information, use normals
                scene.getManhattanDirectionsFromNormals();
            }

        }
    }

    if (scene.has_manhattan_)  {
        updateManhattanDirections(scene.main_dir);
        updateFloorWithManhattan();
    }
}

void GlobalScene::updateManhattanDirections(Eigen::Matrix3f manhattan_dirs) {
    if (!has_manhattan_) {  // If manhattan has never been found, set main_dir
        main_dir = manhattan_dirs;
        has_manhattan_ = true;
    }
    else {

        // Get the angle (as a turn in Y) of Z axis (in floor reference) from old to new reference
        double angle_z = double(acos(manhattan_dirs.col(2).dot(main_dir.col(2))));
        Eigen::Vector3f cross_z = manhattan_dirs.col(2).cross(main_dir.col(2));
        if (main_dir.col(1).dot(cross_z) < 0) { // Or > 0
          angle_z = -angle_z;
        }


        // Keep the axis equally oriented (i.e. Z and X pointing at the same direction) even if you move around
        // Figure in which quadrant the angle_z is, and change main_dir accordingly
        if (angle_z >= M_PI_4 && angle_z < 3*M_PI_4) {
            main_dir.col(0) = -manhattan_dirs.col(2);
            main_dir.col(1) = manhattan_dirs.col(1);
            main_dir.col(2) = manhattan_dirs.col(0);
        }
        else if (angle_z >= 3*M_PI_4 || angle_z <= -3*M_PI_4) {
            main_dir.col(0) = -manhattan_dirs.col(0);
            main_dir.col(1) = manhattan_dirs.col(1);
            main_dir.col(2) = -manhattan_dirs.col(2);
        }
        else if (angle_z > -3*M_PI_4 && angle_z < -M_PI_4) {
            main_dir.col(0) = manhattan_dirs.col(2);
            main_dir.col(1) = manhattan_dirs.col(1);
            main_dir.col(2) = -manhattan_dirs.col(0);
        }
        else {
            main_dir.col(0) = manhattan_dirs.col(0);
            main_dir.col(1) = manhattan_dirs.col(1);
            main_dir.col(2) = manhattan_dirs.col(2);

        }

    }
}

void GlobalScene::updateFloorWithManhattan() {
    floor_normal_.head<3>() = main_dir.col(1);
    c2f.matrix().block<3,3>(0,0) = main_dir.cast<double>().transpose();
    f2c = c2f.inverse();
}



