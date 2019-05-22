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

#include "RGBD/current_scene.h"
//#include "custom_functions.h"

#include <pcl/filters/conditional_removal.h>


void CurrentScene::applyVoxelFilter(float leaf_size, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
	fcloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
	
	pcl::VoxelGrid<pcl::PointXYZ> vg;
	vg.setInputCloud (cloud);
	vg.setLeafSize (leaf_size, leaf_size, leaf_size); 
	vg.filter (*fcloud);
}

void CurrentScene::getAbsoluteCloud(Eigen::Affine3d T) {
    pcl::transformPointCloud(*fcloud,*fcloud,T);
}

void CurrentScene::filterFloorAndCeiling(float floor_height, float ceiling_height) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (fcloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (floor_height, ceiling_height);
    pass.filter (*cloud_filtered);
    pcl::copyPointCloud(*cloud_filtered,*fcloud);
}

void CurrentScene::getNormalsRadius(double radius) {
    normals.reset(new pcl::PointCloud<pcl::Normal>);
    tree =  boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> > (new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod (tree);
    normal_estimator.setInputCloud (fcloud);
    normal_estimator.setRadiusSearch (radius);
    normal_estimator.useSensorOriginAsViewPoint();

    normal_estimator.compute (*normals);
}

void CurrentScene::getNormalsNeighbors(int neighbors) {

    normals.reset(new pcl::PointCloud<pcl::Normal>);
    tree =  boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> > (new pcl::search::KdTree<pcl::PointXYZ>);
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod (tree);
	normal_estimator.setInputCloud (fcloud);
    normal_estimator.setKSearch (neighbors);
    normal_estimator.useSensorOriginAsViewPoint();

    normal_estimator.compute (*normals);

}

bool sortIndicesBySize(const pcl::PointIndices &lhs, const pcl::PointIndices &rhs)
{
    return lhs.indices.size() > rhs.indices.size(); // Large planes to small planes
}


void CurrentScene::regionGrowing() {
    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
    reg.setMinClusterSize (30);
//	reg.setMaxClusterSize (5000000);
    reg.setSmoothModeFlag(true);
	reg.setSearchMethod (tree);
    reg.setNumberOfNeighbours (16);//8
    reg.setInputCloud (fcloud); //fcloud
	reg.setInputNormals (normals);
    reg.setSmoothnessThreshold (6.0 / 180.0 * M_PI);//6
	reg.setCurvatureThreshold (0.5);//0.5

	std::vector <pcl::PointIndices> regions;
	reg.extract (regions);

	pcl::PointIndices::Ptr total_indices (new pcl::PointIndices);
	
    sort(regions.begin(), regions.end(), sortIndicesBySize);
	
	remaining_points.reset(new pcl::PointCloud<pcl::PointXYZ>);
    *remaining_points = *fcloud;//fcloud
	
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> vClusters;
    for (size_t Q=0; Q < regions.size(); Q++) {
        if (regions[Q].indices.size() > 0) {
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp (new pcl::PointCloud<pcl::PointXYZ>());
			pcl::ExtractIndices<pcl::PointXYZ> ex;
            ex.setInputCloud (fcloud);
			pcl::PointIndices::Ptr indices_ (new pcl::PointIndices);
			*indices_ = regions[Q];
			ex.setIndices (indices_);
			ex.setNegative (false);
			ex.filter (*cloud_temp);
			vClusters.push_back(cloud_temp);			
			
            total_indices->indices.insert(total_indices->indices.end(), indices_->indices.begin(), indices_->indices.end());
		}
	}
	
	pcl::ExtractIndices<pcl::PointXYZ> ex;
	ex.setInputCloud (remaining_points);
	ex.setIndices (total_indices);
	ex.setNegative (true);
    ex.filter (*remaining_points);
	
    for (size_t Q=0; Q<vClusters.size(); Q++)	{
        this->isPlane(vClusters[Q]);
	}
	
}



bool CurrentScene::isPlane (pcl::PointCloud<pcl::PointXYZ>::Ptr &region) {
    bool is_plane = false;
    
    if (region->points.size()>30)  {
		// Create the segmentation object for the planar model and set all the parameters
		pcl::SACSegmentation<pcl::PointXYZ> seg;
		pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
		pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
		seg.setOptimizeCoefficients (true);
		seg.setModelType (pcl::SACMODEL_PLANE);
		seg.setMethodType (pcl::SAC_RANSAC);
		seg.setMaxIterations (100);
        seg.setDistanceThreshold (0.04);//0.04
	
		// Segment the largest planar component from the remaining cloud
		seg.setInputCloud (region);
		seg.segment (*inliers, *coefficients);
		
		float n_points_total = region->points.size ();
		float n_points_plane = inliers->indices.size();
		float ratio = n_points_plane/n_points_total;
		
        if (ratio > 0.80f) {
			is_plane = true;
						
            Eigen::Vector4f plane_vector(coefficients->values[0],coefficients->values[1],coefficients->values[2],coefficients->values[3]);
            if (plane_vector[3] < 0) {plane_vector = -plane_vector;}

            Plane plane(region, plane_vector);
				
			vPlanes.push_back(plane);
		}
		else
            vObstacles.push_back(region);

	}
    else
        vObstacles.push_back(region);

	return is_plane;
}

void CurrentScene::extractClusters(pcl::PointCloud<pcl::PointXYZ>::Ptr &points) {
    const int minClusterSize = 50;

    if (points->points.size() > minClusterSize) {
        tree->setInputCloud (points);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance (0.05); // 2cm
        ec.setMinClusterSize (minClusterSize);
//        ec.setMaxClusterSize (5000);
        ec.setSearchMethod (tree);
        ec.setInputCloud (points);
        ec.extract (cluster_indices);

        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster_aux (new pcl::PointCloud<pcl::PointXYZ>);
            for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
                cloud_cluster_aux->points.push_back (points->points[size_t(*pit)]);
            cloud_cluster_aux->width = uint32_t(cloud_cluster_aux->points.size());
            cloud_cluster_aux->height = 1;
            cloud_cluster_aux->is_dense = true;
            vObstacles.push_back(cloud_cluster_aux);
        }
    }
}

void CurrentScene::extractClustersAndPlanes(pcl::PointCloud<pcl::PointXYZ>::Ptr &points) {
    const int minClusterSize = 50;
    if (points->points.size() > minClusterSize) {
        tree->setInputCloud (points);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance (0.03); // 2cm
        ec.setMinClusterSize (minClusterSize);
//        ec.setMaxClusterSize (5000);
        ec.setSearchMethod (tree);
        ec.setInputCloud (points);
        ec.extract (cluster_indices);

        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster_aux (new pcl::PointCloud<pcl::PointXYZ>);
            for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
                cloud_cluster_aux->points.push_back (points->points[size_t(*pit)]);
            cloud_cluster_aux->width = uint32_t(cloud_cluster_aux->points.size());
            cloud_cluster_aux->height = 1;
            cloud_cluster_aux->is_dense = true;

            bool plane_is;
            plane_is = this->isPlane(cloud_cluster_aux);
        }
    }
}

void CurrentScene::getCentroids() {
    for (size_t Q=0; Q<vPlanes.size(); Q++)
        vPlanes[Q].getCentroid();
}

void CurrentScene::getContours() {
    for (size_t Q=0; Q<vPlanes.size(); Q++) {
        vPlanes[Q].getContour();
        vPlanes[Q].fixHolesConcaveHull();
    }
}

void CurrentScene::getPlaneCoeffs2Floor(Eigen::Affine3d c2f) {
    for (size_t Q=0; Q<vPlanes.size();Q++)
        vPlanes[Q].getCoeffs2Floor(c2f);
}

void CurrentScene::getCentroids2Floor(Eigen::Affine3d c2f) {
    for (size_t Q=0; Q<vPlanes.size();Q++) {
        vPlanes[Q].getCentroid2Floor(c2f);
	}
}

void CurrentScene::getClouds2Floor(Eigen::Affine3d c2f) {
    for (size_t Q = 0; Q<vPlanes.size(); Q++)
        vPlanes[Q].getCloud2Floor(c2f);
}

void CurrentScene::transformPlanes(Eigen::Affine3d c2f, bool absolute) {
    if (absolute) {
        for (size_t Q = 0; Q<vPlanes.size(); Q++) {
            vPlanes[Q].coeffs2f = vPlanes[Q].coeffs;
            vPlanes[Q].centroid2f = vPlanes[Q].centroid;
            vPlanes[Q].cloud2f.reset(new pcl::PointCloud<pcl::PointXYZ>());
            *vPlanes[Q].cloud2f = *vPlanes[Q].cloud;
            vPlanes[Q].contour2f.reset(new pcl::PointCloud<pcl::PointXYZ>());
            *vPlanes[Q].contour2f = *vPlanes[Q].contour;
        }
    }
    else {
        for (size_t Q = 0; Q<vPlanes.size(); Q++) {
            vPlanes[Q].getCoeffs2Floor(c2f);
            vPlanes[Q].getCentroid2Floor(c2f);
            vPlanes[Q].getCloud2Floor(c2f);
            vPlanes[Q].getContour2Floor(c2f);
        }
    }
}

void CurrentScene::transformObstacles(Eigen::Affine3d c2f, bool absolute) {

    if (absolute) {
        vObstacles2f = vObstacles;
    }
    else {
        for (size_t Q = 0; Q<vObstacles.size(); Q++) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle (new pcl::PointCloud<pcl::PointXYZ>());
            pcl::transformPointCloud(*vObstacles[Q],*obstacle,c2f);
            vObstacles2f.push_back(obstacle);
        }
    }
}

void CurrentScene::transformPlanesAndObstacles(Eigen::Affine3d c2f, bool absolute) {
    transformPlanes(c2f, absolute);
    transformObstacles(c2f, absolute);
}

void CurrentScene::classifyPlanes() {
    for (size_t Q=0; Q<vPlanes.size();Q++) {
        float A = vPlanes[Q].coeffs2f[0];
        float B = vPlanes[Q].coeffs2f[1];
        float C = vPlanes[Q].coeffs2f[2];
        // float D = vPlanes[Q].coeffs2f[3];

        float dot = vPlanes[Q].coeffs2f.head<3>().dot(Eigen::Vector3f::UnitY());
        dot = ( dot < -1.0f ? -1.0f : ( dot > 1.0f ? 1.0f : dot ) ); // to avoid NaNs
        float angle = pcl::rad2deg(acos(dot));

        if (fabs(A)>fabs(B) and fabs(A)>fabs(C)) {         // If component in X is higher (lateral plane)
            if (fabs(90-angle)<10) // if angle is close to perpendicular to floor normal
                vPlanes[Q].type = 2;
            else
                vPlanes[Q].type = 5;
        }
        else if (fabs(C)>fabs(A) and fabs(C)>fabs(B)) {        // If component in Z is higher (frontal plane)
            if (fabs(90-angle)<10) // if angle is close to perpendicular to floor normal
                vPlanes[Q].type = 3;
            else
                vPlanes[Q].type = 5;
        }
        else if (fabs(B)>fabs(A) and fabs(B)>fabs(C)) {         // If component in Y is higher (horizontal plane)
            if (angle<15){ // || fabs(angle-180) < 15) { // if angle is close to floor normal it is horizontal
                if (fabs(vPlanes[Q].centroid2f.y)<=0.08f) // If close to floor, then it is floor
                    vPlanes[Q].type = 0;
                else
                    vPlanes[Q].type = 1;
            }
            else
                vPlanes[Q].type = 5;
        }
    }
}


void CurrentScene::getManhattanDirectionsFromNormals() {

    int N_normals = int(normals->points.size());
    if (N_normals < 3)
        return;

    Eigen::MatrixXf normals_eigen;
    int N_normals_max = 1000;
    normals_eigen.resize(N_normals_max,3);

    std::vector<size_t> indexes(static_cast<size_t>(N_normals));
    std::iota(std::begin(indexes), std::end(indexes), 0);
    std::random_shuffle(indexes.begin(), indexes.end());

    int size_cloud = 0;
    size_t i = 0;

    while (size_cloud < N_normals_max) {
        if (normals->points[indexes[i]].curvature < 0.005f && normals->points[indexes[i]].curvature > 0.00f) { // curvature must be small, but normals with 0 curvature come from isolated points
            normals_eigen(size_cloud,0) = normals->points[indexes[i]].normal_x;
            normals_eigen(size_cloud,1) = normals->points[indexes[i]].normal_y;
            normals_eigen(size_cloud,2) = normals->points[indexes[i]].normal_z;
            size_cloud++;
        }
        i++;
        if (i == indexes.size())
            break;
    }
    if (size_cloud < N_normals_max) {
        N_normals = size_cloud;
        normals_eigen.resize(N_normals,3);

    }
    else
        N_normals = N_normals_max;


    int N_reps = 500;
    float ang_threshold = 10*float(M_PI)/180;
    int N_inliers_max = 0;

    Eigen::MatrixXf best_eigDx = Eigen::MatrixXf::Identity(3,3);

    for (int i=0; i<N_reps; i++)
    {
        std::vector<int> indexes(static_cast<size_t>(N_normals));
        std::iota(std::begin(indexes), std::end(indexes), 0);
        std::random_shuffle(indexes.begin(), indexes.end());
        indexes.resize(2);

        Eigen::Vector3f v1 = normals_eigen.row(indexes[0]);
        Eigen::Vector3f v_aux = normals_eigen.row(indexes[1]);
        v1.normalize();
        v_aux.normalize();
        Eigen::Vector3f v2 = v1.cross(v_aux);
        v2.normalize();
        Eigen::Vector3f v3 = v1.cross(v2);

        Eigen::MatrixXf ang1, ang2, ang3;
        ang1 = acos((normals_eigen*v1).array().abs());
        ang2 = acos((normals_eigen*v2).array().abs());
        ang3 = acos((normals_eigen*v3).array().abs());
        Eigen::MatrixXf min_ang;
        min_ang = ang3.array().min(ang1.array().min(ang2.array()));

        int N_inliers = int((min_ang.array() < ang_threshold).count());

        if (N_inliers > N_inliers_max) {
           N_inliers_max = N_inliers;
           Eigen::MatrixXf current_eigDx(3,3);
           current_eigDx << v1, v2, v3;
           best_eigDx = current_eigDx;

           if (N_inliers_max > 0.60f*float(N_normals)) {
               has_manhattan = true;
                break;
           }
        }
        if (N_inliers_max > 0.3f*float(N_normals) && i > 50) {
            has_manhattan = true;
            break;
        }
    }


    if (has_manhattan)
        eigDx = sortManhattanDirections(best_eigDx);

}




void CurrentScene::getManhattanDirectionsFromPlanesWithFloor(Eigen::Affine3d f2c) {
    int n_inliers = 0;
    int max_n_inliers = 0;
    int total_points = 0;
    int plane_selected = 0;
    Eigen::Vector3f best_normal(0,0,0);

    for (size_t P = 0; P < vPlanes.size(); P++) {
        total_points += vPlanes[P].cloud->points.size();
        if (vPlanes[P].type == 2 || vPlanes[P].type == 3) {
            Eigen::Vector3f normal;
            normal = vPlanes[P].coeffs2f.block<3,1>(0,0);

            n_inliers = int(vPlanes[P].cloud->points.size());

            for (size_t Q = 0; Q < vPlanes.size(); Q++) {
                if ((Q != P) and (vPlanes[Q].type == 2 || vPlanes[Q].type == 3)) {
                    Eigen::Vector3f current_normal;
                    current_normal = vPlanes[Q].coeffs2f.block<3,1>(0,0);
                    double angle = double(acos(Eigen::Vector2f(normal(0),normal(2)).normalized().dot(Eigen::Vector2f(current_normal(0),current_normal(2)))));

                    double threshold = 5.0*M_PI/180.0;
                    if ((fabs(2*M_PI-angle) < threshold) or (fabs(M_PI/2-angle) < threshold) or (fabs(M_PI-angle) < threshold) or (fabs(3/2*M_PI-angle) < threshold) or (angle < threshold)) {
                        n_inliers += vPlanes[Q].cloud->points.size();
                    }
                }
            }

            if (n_inliers > max_n_inliers) {
                max_n_inliers = n_inliers;
                best_normal = normal;
                plane_selected = int(P);
            }
        }
    }


    if (max_n_inliers > 0){
		
        has_manhattan = true;

        Eigen::Matrix3f best_eigDx;
        best_eigDx.col(0) = (f2c.rotation().cast<float>())*Eigen::Vector3f(best_normal(2),0, best_normal(0));
        best_eigDx.col(1) = (f2c.rotation().cast<float>())*Eigen::Vector3f(0,1,0);
        best_eigDx.col(2) = (f2c.rotation().cast<float>())*Eigen::Vector3f(best_normal(0),0,best_normal(2));

        eigDx = sortManhattanDirections(best_eigDx);
    }
}

void CurrentScene::getManhattanDirectionsFromPlanes()  {

    int N_inliers_max = 0;
    const float ang_threshold = float(5*M_PI/180);
    Eigen::MatrixXf best_eigDx = Eigen::MatrixXf::Identity(3,3);

    Eigen::MatrixXf normals_eigen(vPlanes.size(),3);
    for (size_t P = 0; P < vPlanes.size(); P++)
        normals_eigen.block<1,3>(P,0) = vPlanes[P].coeffs.block<3,1>(0,0);

    for (size_t P = 0; P < vPlanes.size(); P++) {
        for (size_t Q = 0; Q < vPlanes.size(); Q++)  {
            if ((vPlanes[P].type != vPlanes[Q].type) && !(vPlanes[P].type <= 1 && vPlanes[Q].type <= 1)) {
                int N_inliers = 0;
                int N_planes = 0;

                Eigen::Vector3f normal_1, normal_2;
                normal_1 = normals_eigen.row(P);
                normal_2 = normals_eigen.row(Q);

                Eigen::Vector3f v1 = normals_eigen.row(P);
                Eigen::Vector3f v_aux = normals_eigen.row(Q);
                Eigen::Vector3f v2 = v1.cross(v_aux);//normals->points[indexes[1]].getNormalVector3fMap());
                v2.normalize();
                Eigen::Vector3f v3 = v1.cross(v2);

                Eigen::MatrixXf ang1, ang2, ang3;
                ang1 = acos((normals_eigen*v1).array().abs());
                ang2 = acos((normals_eigen*v2).array().abs());
                ang3 = acos((normals_eigen*v3).array().abs());
                Eigen::MatrixXf min_ang;
                min_ang = ang3.array().min(ang1.array().min(ang2.array()));

                for (size_t i = 0; i< vPlanes.size(); i++) {
                    if (min_ang(i,0) < ang_threshold) {
                        N_inliers += vPlanes[i].cloud->points.size();
                        N_planes++;
                    }
                }

                if (N_inliers > N_inliers_max && N_planes > 1) {
                    N_inliers_max = N_inliers;
                    Eigen::MatrixXf current_eigDx(3,3);
                    current_eigDx << v1, v2, v3;
                    best_eigDx = current_eigDx;
                }
            }

        }
    }

    if (N_inliers_max > 0) {
        eigDx = sortManhattanDirections(best_eigDx);
        has_manhattan = true;
    }
}


Eigen::Matrix3f CurrentScene::sortManhattanDirections(Eigen::Matrix3f best_eigDx) {
    Eigen::Vector3f v_floor(0.0f, -sin(float(M_PI_4)), -sin(float(M_PI_4)));
    Eigen::Vector3f v_front(0.0f, 0.0f, 1.0f);

    Eigen::MatrixXf angles_y, angles_z;
    angles_y = acos((v_floor.transpose()*best_eigDx).array().abs());
    angles_z = acos((v_front.transpose()*best_eigDx).array().abs());

    int dummy, index_x, index_y, index_z;
    float min_y, min_z;

    min_y = angles_y.minCoeff(&dummy,&index_y);
    min_z = angles_z.minCoeff(&dummy,&index_z);

    if (index_y == index_z){
        if (min_y < min_z){
            if ((index_y != 0) && (angles_z(0,0) < angles_z(0,1) || angles_z(0,0) < angles_z(0,2)))
                    index_z = 0;
            else if ((index_y != 1) && (angles_z(0,1) < angles_z(0,0) || angles_z(0,1) < angles_z(0,2)))
                    index_z = 1;
            else if ((index_y != 2) && (angles_z(0,2) < angles_z(0,0) || angles_z(0,2) < angles_z(0,1)))
                    index_z = 2;
        }
        else{
            if ((index_z != 0) && (angles_y(0,0) < angles_y(0,1) || angles_y(0,0) < angles_y(0,2)))
                    index_y = 0;
            if ((index_z != 1) && (angles_y(0,1) < angles_y(0,0) || angles_y(0,1) < angles_y(0,2)))
                    index_y = 1;
            if ((index_z != 2) && (angles_y(0,2) < angles_y(0,0) || angles_y(0,2) < angles_y(0,1)))
                    index_y = 2;
        }
    }

    if (index_y == 0)
        index_z == 1 ? index_x = 2 : index_x = 1;
    else if (index_y == 1)
        index_z == 1 ? index_x = 2 : index_x = 0;
    else
        index_z == 0 ? index_x = 1 : index_x = 0;

    Eigen::Vector3f dx = best_eigDx.col(index_x);
    Eigen::Vector3f dy = best_eigDx.col(index_y);
    Eigen::Vector3f dz = best_eigDx.col(index_z);

    if (dy(1) > 0)
        dy = -dy;
    if (dz(2) < 0)
        dz = -dz;
    dx = dy.cross(dz);

    Eigen::MatrixXf final_eigDx(3,3);
    final_eigDx << dx, dy, dz;

    return final_eigDx;
}


