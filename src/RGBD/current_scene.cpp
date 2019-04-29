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


void CurrentScene::applyVoxelFilter(float leaf_size, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	fcloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
	
	pcl::VoxelGrid<pcl::PointXYZ> vg;
	vg.setInputCloud (cloud);
	vg.setLeafSize (leaf_size, leaf_size, leaf_size); 
	vg.filter (*fcloud);
}

void CurrentScene::getAbsoluteCloud(Eigen::Affine3d T)
{
//    pcl::PointCloud<pcl::PointXYZ>::Ptr aux (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*fcloud,*fcloud,T);
//    pcl::copyPointCloud(*aux,*fcloud);
}

void CurrentScene::filterFloorAndCeiling(double floor_height, double ceiling_height)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (fcloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (floor_height, ceiling_height);
    pass.filter (*cloud_filtered);
    pcl::copyPointCloud(*cloud_filtered,*fcloud);
}

void CurrentScene::getNormalsRadius(float radius)
{
    pcl::PointCloud<pcl::Normal>::Ptr normals_aux(new pcl::PointCloud<pcl::Normal>);
    pcl::search::Search<pcl::PointXYZ>::Ptr tree_aux =  boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> > (new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod (tree_aux);
    normal_estimator.setInputCloud (fcloud);
    normal_estimator.setRadiusSearch (radius);
    normal_estimator.useSensorOriginAsViewPoint();

    normal_estimator.compute (*normals_aux);

    normals.reset(new pcl::PointCloud<pcl::Normal>);
    *normals = *normals_aux;

    tree =  boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> > (new pcl::search::KdTree<pcl::PointXYZ>);
    *tree = *tree_aux;
}

void CurrentScene::getNormalsNeighbors(int neighbors)
{
	pcl::PointCloud<pcl::Normal>::Ptr normals_aux(new pcl::PointCloud<pcl::Normal>);
	pcl::search::Search<pcl::PointXYZ>::Ptr tree_aux =  boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> > (new pcl::search::KdTree<pcl::PointXYZ>);
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
	normal_estimator.setSearchMethod (tree_aux);
	normal_estimator.setInputCloud (fcloud);
  normal_estimator.setKSearch (neighbors);
  normal_estimator.useSensorOriginAsViewPoint();

	normal_estimator.compute (*normals_aux);
	
	normals.reset(new pcl::PointCloud<pcl::Normal>);
	*normals = *normals_aux;

  tree =  boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> > (new pcl::search::KdTree<pcl::PointXYZ>);
	*tree = *tree_aux;
}


void CurrentScene::regionGrowing()
{
	// Extraer regiones con el algoritmo region growing del pcl
	pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
        reg.setMinClusterSize (50);
	// reg.setMinClusterSize (20);
	reg.setMaxClusterSize (5000000);
	reg.setSmoothModeFlag(true);
	reg.setSearchMethod (tree);
        reg.setNumberOfNeighbours (8);//8
        reg.setInputCloud (fcloud); //fcloud
	reg.setInputNormals (normals);
    reg.setSmoothnessThreshold (10.0 / 180.0 * M_PI);//6
	reg.setCurvatureThreshold (0.5);//0.5

	std::vector <pcl::PointIndices> regions;
	reg.extract (regions);

	pcl::PointIndices::Ptr total_indices (new pcl::PointIndices);
	
	//// Ordenar las regiones por tamaño
	//sort(regions.begin(), regions.end(), sortIndicesBySize);
	
	remaining_points.reset(new pcl::PointCloud<pcl::PointXYZ>);
        *remaining_points = *fcloud;//fcloud
	
	// Crear el vector de clusters que contiene todas las regiones encontradas
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> vClusters;
	for (int Q=0; Q < regions.size(); Q++)
	{
		if (regions[Q].indices.size() > 0)
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp (new pcl::PointCloud<pcl::PointXYZ>());
			pcl::ExtractIndices<pcl::PointXYZ> ex;
                        ex.setInputCloud (fcloud);//fcloud
			pcl::PointIndices::Ptr indices_ (new pcl::PointIndices);
			*indices_ = regions[Q];
			ex.setIndices (indices_);
			ex.setNegative (false);
			ex.filter (*cloud_temp);
			vClusters.push_back(cloud_temp);			
			
			total_indices->indices.insert(total_indices->indices.end(), indices_->indices.begin(), indices_->indices.end());
			
			//total_indices->indices.push_back(indices_->indices);
			//for (int P = 0; P < regions[Q].indices.size(); P++)
			//{
			//	total_indices->indices.push_back(regions[Q].indices[P]);
			//}
			////pcl::copyPointCloud(*total_indices,*indices_,*total_indices);
		}
	}
	
	pcl::ExtractIndices<pcl::PointXYZ> ex;
	ex.setInputCloud (remaining_points);
	ex.setIndices (total_indices);
	ex.setNegative (true);
	ex.filter (*remaining_points);
	
	//pcl::ExtractIndices<pcl::Normal> ex_normal;
        //ex_normal.setInputCloud (normals);99999
	//ex_normal.setIndices (total_indices);
	//ex_normal.setNegative (true);
	//ex_normal.filter (*remaining_normals);
	
	// Análisis de la planitud de los clusters encontrados
	for (int Q=0; Q<vClusters.size(); Q++)
	{
		bool plane_is;
		plane_is = this->isPlane(vClusters[Q]);	
	}
	
}



bool CurrentScene::isPlane (pcl::PointCloud<pcl::PointXYZ>::Ptr &region)
{
    bool is_plane = false;
    
    if (region->points.size()>30)//50
    {
		// Create the segmentation object for the planar model and set all the parameters
		pcl::SACSegmentation<pcl::PointXYZ> seg;
		pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
		pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
		seg.setOptimizeCoefficients (true);
		seg.setModelType (pcl::SACMODEL_PLANE);
		seg.setMethodType (pcl::SAC_RANSAC);
		seg.setMaxIterations (100);
		seg.setDistanceThreshold (0.02);//0.04
	
		// Segment the largest planar component from the remaining cloud
		seg.setInputCloud (region);
		seg.segment (*inliers, *coefficients);
		
		float n_points_total = region->points.size ();
		float n_points_plane = inliers->indices.size();
		float ratio = n_points_plane/n_points_total;
		
		if (ratio > 0.80)//0.8
		{
			is_plane = true;
						
			Eigen::Vector4f plane_vector(coefficients->values[0],coefficients->values[1],coefficients->values[2],coefficients->values[3]);
			//double angle = pcl::getAngle3D(plane_vector,viewpoint);
			double angle = pcl::getAngle3D(plane_vector,Eigen::Vector4f(0,2,2,1));
			if (pcl::rad2deg(angle) > 90)
				plane_vector = -plane_vector;
		

      Plane plane(region, plane_vector);
//			plane.coeffs = plane_vector;
//			plane.cloud.reset(new pcl::PointCloud<pcl::PointXYZ>() );
//			*plane.cloud = *region;
				
			vPlanes.push_back(plane);
		}
		else
		{
			vObstacles.push_back(region);
		}
	}
	else
	{
		vObstacles.push_back(region);
	}
	return is_plane;
}

void CurrentScene::extractClusters(pcl::PointCloud<pcl::PointXYZ>::Ptr &points)
{
    int minClusterSize = 50;

    if (points->points.size() > minClusterSize)
    {
        tree->setInputCloud (points);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance (0.05); // 2cm
        ec.setMinClusterSize (minClusterSize);
        ec.setMaxClusterSize (5000);
        ec.setSearchMethod (tree);
        ec.setInputCloud (points);
        ec.extract (cluster_indices);

        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster_aux (new pcl::PointCloud<pcl::PointXYZ>);
            for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
                cloud_cluster_aux->points.push_back (points->points[*pit]);
            cloud_cluster_aux->width = cloud_cluster_aux->points.size ();
            cloud_cluster_aux->height = 1;
            cloud_cluster_aux->is_dense = true;
            vObstacles.push_back(cloud_cluster_aux);
        }
    }
}

void CurrentScene::extractClustersAndPlanes(pcl::PointCloud<pcl::PointXYZ>::Ptr &points)
{
	tree->setInputCloud (points);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance (0.03); // 2cm
	ec.setMinClusterSize (30);
	ec.setMaxClusterSize (5000);
	ec.setSearchMethod (tree);
	ec.setInputCloud (points);
	ec.extract (cluster_indices);

	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster_aux (new pcl::PointCloud<pcl::PointXYZ>);
		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
			cloud_cluster_aux->points.push_back (points->points[*pit]);
		cloud_cluster_aux->width = cloud_cluster_aux->points.size ();
		cloud_cluster_aux->height = 1;
		cloud_cluster_aux->is_dense = true;
		
		bool plane_is;
		plane_is = this->isPlane(cloud_cluster_aux);	
	}
}

void CurrentScene::getCentroids()
{
  for (int Q=0; Q<vPlanes.size(); Q++)
  {
    vPlanes[Q].getCentroid();
  }
}

void CurrentScene::getContours()
{
    for (int Q=0; Q<vPlanes.size(); Q++)
    {
        vPlanes[Q].getContour();
        vPlanes[Q].fixHolesConcaveHull();
    }
}

void CurrentScene::getPlaneCoeffs2Floor(Eigen::Affine3d c2f)
{
	for (int Q=0; Q<vPlanes.size();Q++)
	{
//		Eigen::Affine3f c2f_aux = (Eigen::Affine3f) c2f;
//		Eigen::Vector3f translation = Eigen::Vector3f(0,0,0);
//		c2f_aux.translation() = translation;
//		vPlanes[Q].coeffs2f = (c2f_aux)*vPlanes[Q].coeffs;
				
//		double angle = pcl::getAngle3D(vPlanes[Q].coeffs2f,viewpoint);
//		if (pcl::rad2deg(angle) > 90)
//			vPlanes[Q].coeffs = -vPlanes[Q].coeffs;

//    vPlanes[Q].coeffs2f = c2f.inverse().matrix().transpose().cast<float>()*vPlanes[Q].coeffs;

    vPlanes[Q].getCoeffs2Floor(c2f);
	}
}

void CurrentScene::getCentroids2Floor(Eigen::Affine3d c2f)
{	
	for (int Q=0; Q<vPlanes.size();Q++)
	{
		vPlanes[Q].getCentroid2Floor(c2f);
		//vPlanes[Q].centroid2f.x = static_cast<float> (c2f.matrix() (0, 0) * vPlanes[Q].centroid.x + c2f.matrix() (0, 1) * vPlanes[Q].centroid.y + c2f.matrix() (0, 2) * vPlanes[Q].centroid.z + c2f.matrix() (0, 3));
		//vPlanes[Q].centroid2f.y = static_cast<float> (c2f.matrix() (1, 0) * vPlanes[Q].centroid.x + c2f.matrix() (1, 1) * vPlanes[Q].centroid.y + c2f.matrix() (1, 2) * vPlanes[Q].centroid.z + c2f.matrix() (1, 3));
		//vPlanes[Q].centroid2f.z = static_cast<float> (c2f.matrix() (2, 0) * vPlanes[Q].centroid.x + c2f.matrix() (2, 1) * vPlanes[Q].centroid.y + c2f.matrix() (2, 2) * vPlanes[Q].centroid.z + c2f.matrix() (2, 3));

		//vPlanes[Q].centroid2f = pcl::transformPoint(vPlanes[Q].centroid,c2f.rotation());	

	}
}

void CurrentScene::getClouds2Floor(Eigen::Affine3d c2f)
{
  for (int Q = 0; Q<vPlanes.size(); Q++)
  {
    vPlanes[Q].getCloud2Floor(c2f);
  }
}

void CurrentScene::transformPlanes(Eigen::Affine3d c2f, bool absolute)
{

  if (absolute)
  {
    for (int Q = 0; Q<vPlanes.size(); Q++)
    {
      vPlanes[Q].coeffs2f = vPlanes[Q].coeffs;
      vPlanes[Q].centroid2f = vPlanes[Q].centroid;
      vPlanes[Q].cloud2f.reset(new pcl::PointCloud<pcl::PointXYZ>());
      *vPlanes[Q].cloud2f = *vPlanes[Q].cloud;
      vPlanes[Q].contour2f.reset(new pcl::PointCloud<pcl::PointXYZ>());
      *vPlanes[Q].contour2f = *vPlanes[Q].contour;
    }
  }
  else
  {
    for (int Q = 0; Q<vPlanes.size(); Q++)
    {
//      Eigen::Affine3f c2f_aux = (Eigen::Affine3f) c2f;
//      Eigen::Vector3f translation = Eigen::Vector3f(0,0,0);
//      c2f_aux.translation() = translation;
//      vPlanes[Q].coeffs2f = (c2f_aux)*vPlanes[Q].coeffs;

//      vPlanes[Q].coeffs2f = c2f.inverse().matrix().transpose().cast<float>()*vPlanes[Q].coeffs;

//      double angle = pcl::getAngle3D(vPlanes[Q].coeffs2f,viewpoint);
//      if (pcl::rad2deg(angle) > 90)
//        vPlanes[Q].coeffs = -vPlanes[Q].coeffs;

      vPlanes[Q].getCoeffs2Floor(c2f);
      vPlanes[Q].getCentroid2Floor(c2f);
      vPlanes[Q].getCloud2Floor(c2f);
      vPlanes[Q].getContour2Floor(c2f);
    }
  }
}

void CurrentScene::transformObstacles(Eigen::Affine3d c2f, bool absolute)
{

  if (absolute)
  {
    vObstacles2f = vObstacles;
  }
  else
  {
    for (int Q = 0; Q<vObstacles.size(); Q++)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle (new pcl::PointCloud<pcl::PointXYZ>());
      pcl::transformPointCloud(*vObstacles[Q],*obstacle,c2f);
      vObstacles2f.push_back(obstacle);
    }
  }
}

void CurrentScene::transformPlanesAndObstacles(Eigen::Affine3d c2f, bool absolute)
{
  transformPlanes(c2f, absolute);
  transformObstacles(c2f, absolute);
}

void CurrentScene::classifyPlanes()
{
        for (int Q=0; Q<vPlanes.size();Q++)
        {
                float A = -vPlanes[Q].coeffs2f[0];
                float B = -vPlanes[Q].coeffs2f[1];
                float C = -vPlanes[Q].coeffs2f[2];
                float D = -vPlanes[Q].coeffs2f[3];

//                std::cout << A << " " << B << " " << C << " " << D << std::endl;

//                float angle = fabs(getAngle(Eigen::Vector4f(A,B,C,D), Eigen::Vector4f(0,1,0,0))*180/M_PI);
                float angle = acos(Eigen::Vector3f(A,B,C).dot(Eigen::Vector3f(0,1,0)));


//                std::cout << angle << std::endl;

                //Identificación de planos
                if (fabs(A)>fabs(B) and fabs(A)>fabs(C))
                {
                        if (D>0)
                        {
                                if (fabs(90-angle)<10)
                                {
                                        // Pared derecha
                                        vPlanes[Q].type = 2;
                                }
                                else
                                {
                                        // Obstáculo derecho
                                        vPlanes[Q].type = 5;
                                }
                        }
                        else
                        {
                                if (fabs(90-angle)<10)
                                {
                                        // Pared izquierda
                                        vPlanes[Q].type = 2;
                                }
                                else
                                {
                                        // Obstáculo izquierdo
                                        vPlanes[Q].type = 5;
                                }
                        }
                }
                else if (fabs(C)>fabs(A) and fabs(C)>fabs(B))
                {
                        if (fabs(90-angle)<10)
                        {
                                // Pared frontal
                                vPlanes[Q].type = 3;
                        }
                        else
                        {
                                // Obstáculo frontal
                                vPlanes[Q].type = 5;
                        }
                }
                else if (fabs(B)>fabs(A) and fabs(B)>fabs(C))
                {
                        // Suelo
                        if ((angle<15) || fabs(angle-180) < 15)
                        // if (angle<10)
                        {
//                                 std::cout << fabs(vPlanes[Q].centroid2f.y) << std::endl;
                                if (fabs(vPlanes[Q].centroid2f.y)<=0.08f)
                                // if (fabs(vPlanes[Q].centroid2f.y)<=0.07f)
                                {
                                        vPlanes[Q].type = 0;
                                }
                                else
                                {
                                        vPlanes[Q].type = 1;
                                }
                        }
                        else
                                vPlanes[Q].type = 5;
                }
        }
        //std::cout << std::endl;
}

void CurrentScene::classifyPlanesAbs()
{
        for (int Q=0; Q<vPlanes.size();Q++)
        {
                float A = -vPlanes[Q].coeffs2f[0];
                float B = -vPlanes[Q].coeffs2f[1];
                float C = -vPlanes[Q].coeffs2f[2];
                float D = -vPlanes[Q].coeffs2f[3];

//                std::cout << A << " " << B << " " << C << " " << D << std::endl;

//                float angle = fabs(getAngle(Eigen::Vector4f(A,B,C,D), Eigen::Vector4f(0,0,1,0))*180/M_PI);
                float angle = acos(Eigen::Vector3f(A,B,C).dot(Eigen::Vector3f(0,1,0)));


//                std::cout << angle << std::endl;

                //Identificación de planos
                if (fabs(B)>fabs(A) and fabs(B)>fabs(A))
                {
                  if (fabs(90-angle)<10)
                  {
                    // Pared izquierda o derecha
                    vPlanes[Q].type = 2;
                  }
                  else
                  {
                    // Obstáculo izquierdo o derecho
                    vPlanes[Q].type = 5;
                  }
                }
                else if (fabs(A)>fabs(B) and fabs(A)>fabs(C))
                {
                        if (fabs(90-angle)<10)
                        {
                                // Pared frontal
                                vPlanes[Q].type = 3;
                        }
                        else
                        {
                                // Obstáculo frontal
                                vPlanes[Q].type = 5;
                        }
                }
                else if (fabs(C)>fabs(A) and fabs(C)>fabs(B))
                {
                        // Suelo
                        if ((angle<15) || fabs(angle-180) < 15)
                        // if (angle<10)
                        {
//                                 std::cout << fabs(vPlanes[Q].centroid2f.y) << std::endl;
                                if (fabs(vPlanes[Q].centroid2f.z)<=0.08f)
                                // if (fabs(vPlanes[Q].centroid2f.y)<=0.07f)
                                {
                                        vPlanes[Q].type = 0;
                                }
                                else
                                {
                                        vPlanes[Q].type = 1;
                                }
                        }
                        else
                                vPlanes[Q].type = 5;
                }
        }
        //std::cout << std::endl;
}








void CurrentScene::getManhattanDirectionsFromNormals()
{

    size_t N_normals = normals->points.size();
    if (N_normals < 3)
        return;

    /* //Conditional removal (not working on PCL 1.7... internet suggest to try on 1.8)
    pcl::ConditionAnd<pcl::Normal>::Ptr curvature_cond (new pcl::ConditionAnd<pcl::Normal>());
    curvature_cond->addComparison(pcl::FieldComparison<pcl::Normal>::ConstPtr (new pcl::FieldComparison<pcl::Normal>("curvature", pcl::ComparisonOps::GT, 0.05)));
    pcl::ConditionalRemoval<pcl::Normal> condrem;
    condrem.setCondition(curvature_cond);
    condrem.setInputCloud(normals);
    condrem.filter(*normals_aux);


    N_normals = normals_aux->points.size();
    if (N_normals < 3)
        return;
    */
    Eigen::MatrixXf normals_eigen;
    size_t N_normals_max = 1000;
    normals_eigen.resize(N_normals_max,3);

//    if (N_normals > N_normals_max)
//    {
        std::vector<int> indexes(N_normals);
        std::iota(std::begin(indexes), std::end(indexes), 0);
        std::random_shuffle(indexes.begin(), indexes.end());
//        indexes.resize(N_normals_max);

        size_t size_cloud = 0;
        int i = 0;

        while (size_cloud < N_normals_max)
        {
            if (normals->points[indexes[i]].curvature < 0.005)
            {
                normals_eigen(size_cloud,0) = normals->points[indexes[i]].normal_x;
                normals_eigen(size_cloud,1) = normals->points[indexes[i]].normal_y;
                normals_eigen(size_cloud,2) = normals->points[indexes[i]].normal_z;
                size_cloud++;
            }
            i++;
            if (i == indexes.size())
                break;
        }
        if (size_cloud < N_normals_max)
        {
            N_normals = size_cloud;
            normals_eigen.resize(N_normals,3);

        }
        else
            N_normals = N_normals_max;



    /* PCA ATTEMPT
    std::cout << normals_eigen << std::endl << std::endl;
//    MatrixXd centered = mat.rowwise() - mat.colwise().mean();
//    MatrixXd cov = (centered.adjoint() * centered) / double(mat.rows() - 1);
    Eigen::MatrixXf covariance = (normals_eigen.adjoint() * normals_eigen)/float(normals_eigen.rows()-1);
    std::cout << covariance  << std::endl << std::endl;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance,Eigen::ComputeEigenvectors);
    eigDx = eigen_solver.eigenvectors();
    eigDx.col(2) = eigDx.col(0).cross(eigDx.col(1));
    std::cout << eigDx << std::endl << std::endl;*/

    int N_reps = 500;
    float ang_threshold = 10*M_PI/180;
    int N_inliers_max = 0;
    float sum_angles_min = M_PI/2*N_normals;

    Eigen::MatrixXf best_eigDx = Eigen::MatrixXf::Identity(3,3);

    for (int i=0; i<N_reps; i++)
    {
        std::vector<int> indexes(N_normals);
        std::iota(std::begin(indexes), std::end(indexes), 0);
        std::random_shuffle(indexes.begin(), indexes.end());
        indexes.resize(2);

        Eigen::Vector3f v1 = normals_eigen.row(indexes[0]);//normals->points[indexes[0]].getNormalVector3fMap();
        Eigen::Vector3f v_aux = normals_eigen.row(indexes[1]);
        Eigen::Vector3f v2 = v1.cross(v_aux);//normals->points[indexes[1]].getNormalVector3fMap());
        v2.normalize();
        Eigen::Vector3f v3 = v1.cross(v2);

        Eigen::MatrixXf ang1, ang2, ang3;
        ang1 = acos((normals_eigen*v1).array().abs());
        ang2 = acos((normals_eigen*v2).array().abs());
        ang3 = acos((normals_eigen*v3).array().abs());
        Eigen::MatrixXf min_ang;
        min_ang = ang3.array().min(ang1.array().min(ang2.array()));

        float sum_angles = min_ang.sum();

        float N_inliers = (min_ang.array() < ang_threshold).count();

        if (N_inliers > N_inliers_max)
//        if (sum_angles < sum_angles_min)
        {
           N_inliers_max = N_inliers;
           sum_angles_min = sum_angles;
           Eigen::MatrixXf current_eigDx(3,3);
           current_eigDx << v1, v2, v3;
           best_eigDx = current_eigDx;

//           std::cout << "current eigDx\n" << current_eigDx << std::endl;
//           std::cout << "sum_angles: " << sum_angles << std::endl;
//           std::cout << "N_inliers: " << N_inliers << " N_normals: " << N_normals << std::endl;

           if (N_inliers_max > 0.80*float(N_normals))
           {
               has_manhattan = true;
                break;
           }
        }
         if (N_inliers_max > 0.5*float(N_normals) && i > 50)
         {
             has_manhattan = true;
             break;
         }

    }

    if (has_manhattan)
        eigDx = sortManhattanDirections(best_eigDx);


}




void CurrentScene::getManhattanDirectionsFromPlanesWithFloor(Eigen::Affine3d f2c)
{
    int n_inliers = 0;
    int max_n_inliers = 0;
    int total_points = 0;
    int plane_selected = 0;
    Eigen::Vector3f best_normal(0,0,0);

    for (int P = 0; P < vPlanes.size(); P++)
    {
        total_points += vPlanes[P].cloud->points.size();
        if (vPlanes[P].type == 2 || vPlanes[P].type == 3)
        {
            Eigen::Vector3f normal;
            normal = vPlanes[P].coeffs2f.block<3,1>(0,0);

            n_inliers = vPlanes[P].cloud->points.size();

            for (int Q = 0; Q < vPlanes.size(); Q++)
            {
                if ((Q != P) and (vPlanes[Q].type == 2 || vPlanes[Q].type == 3))
                {
                    Eigen::Vector3f current_normal;
                    current_normal = vPlanes[Q].coeffs2f.block<3,1>(0,0);
//                    float angle;
//                    angle = getHorizontalAngle(normal,current_normal);
                    float angle = acos(Eigen::Vector2f(normal(0),normal(2)).normalized().dot(Eigen::Vector2f(current_normal(0),current_normal(2))));


                    float threshold = 5*M_PI/180;
                    if ((fabs(2*M_PI-angle) < threshold) or (fabs(M_PI/2-angle) < threshold) or (fabs(M_PI-angle) < threshold) or (fabs(3/2*M_PI-angle) < threshold) or (angle < threshold))
                    {
                        n_inliers += vPlanes[Q].cloud->points.size();
                    }
                }
            }

            if (n_inliers > max_n_inliers)
            {
                max_n_inliers = n_inliers;
                best_normal = normal;
                plane_selected = P;
            }
        }
    }

//    if (max_n_inliers > 0.1*((float)total_points))
    if (max_n_inliers > 0)
	{
		
        has_manhattan = true;
//        Eigen::Matrix3f eigDx2f;

//        eigDx2f.col(0) = Eigen::Vector3f(best_normal(2),0, best_normal(0));
//        eigDx2f.col(1) = Eigen::Vector3f(0,1,0);
//        eigDx2f.col(2) = Eigen::Vector3f(best_normal(0),0,best_normal(2));
	
////		float angle_x = getHorizontalAngle(best_normal,Eigen::Vector3f(1,0,0));
////		float angle_z = getHorizontalAngle(best_normal,Eigen::Vector3f(0,0,1));
//        float angle_x = acos(Eigen::Vector2f(best_normal(0),best_normal(2)).normalized().dot(Eigen::Vector2f(1,0)));
//        float angle_z = acos(Eigen::Vector2f(best_normal(0),best_normal(2)).normalized().dot(Eigen::Vector2f(0,1)));

//		float threshold = 45*M_PI/180;

//		if ((fabs(2*M_PI-angle_z) < threshold) or (angle_z < threshold))
//		{
//			// std::cout << "1" << std::endl << std::endl;
//			eigDx2f.col(0) = Eigen::Vector3f(best_normal(2),0, -best_normal(0));
//			eigDx2f.col(1) = Eigen::Vector3f(0,1,0);
//			eigDx2f.col(2) = Eigen::Vector3f(best_normal(0),0,best_normal(2));
//		}
//		else if (fabs(M_PI-angle_z) < threshold)
//		{
//			// std::cout << "2" << std::endl << std::endl;
//			eigDx2f.col(0) = Eigen::Vector3f(-best_normal(2),0,best_normal(0));
//			eigDx2f.col(1) = Eigen::Vector3f(0,1,0);
//			eigDx2f.col(2) = Eigen::Vector3f(-best_normal(0),0,-best_normal(2));
//		}
//		else
//		{
//			if ((fabs(2*M_PI-angle_x) < threshold) or (angle_x < threshold))
//			{
//				// std::cout << "3" << std::endl << std::endl;
//				eigDx2f.col(0) = Eigen::Vector3f(best_normal(0),0, best_normal(2));
//				eigDx2f.col(1) = Eigen::Vector3f(0,1,0);
//				eigDx2f.col(2) = Eigen::Vector3f(-best_normal(2),0, best_normal(0));
//			}
//			else if (fabs(M_PI-angle_x) < threshold)
//			{
//				// std::cout << "4" << std::endl << std::endl;
//				eigDx2f.col(0) = Eigen::Vector3f(-best_normal(0),0,-best_normal(2));
//				eigDx2f.col(1) = Eigen::Vector3f(0,1,0);
//				eigDx2f.col(2) = Eigen::Vector3f(best_normal(2),0,-best_normal(0));
//			}
//			else
//			{
//				// std::cout << "solución genérica" << std::endl << std::endl;
//				eigDx2f.col(0) = Eigen::Vector3f(best_normal(0),0,best_normal(2));
//				eigDx2f.col(1) = Eigen::Vector3f(0,1,0);
//				eigDx2f.col(2) = Eigen::Vector3f(-best_normal(2),0,best_normal(0));
//			}
//		}


//    Eigen::Affine3f f2c_aux = (Eigen::Affine3f) f2c;
//		Eigen::Vector3f translation = Eigen::Vector3f(0,0,0);
//    f2c_aux.translation().setZero();// = Eigen::Vector3f(0,0,0);


    Eigen::Matrix3f best_eigDx;
    best_eigDx.col(0) = (f2c.rotation().cast<float>())*Eigen::Vector3f(best_normal(2),0, best_normal(0));
    best_eigDx.col(1) = (f2c.rotation().cast<float>())*Eigen::Vector3f(0,1,0);
    best_eigDx.col(2) = (f2c.rotation().cast<float>())*Eigen::Vector3f(best_normal(0),0,best_normal(2));

    eigDx = sortManhattanDirections(best_eigDx);
	}
//	else
//        has_manhattan = false;
}

void CurrentScene::getManhattanDirectionsFromPlanes()
{

    int N_inliers_max = 0;
//    int total_points = 0;
//    int plane_selected = 0;
//    Eigen::Vector3f best_normal(0,0,0);
    float ang_threshold = 5*M_PI/180;
    Eigen::MatrixXf best_eigDx = Eigen::MatrixXf::Identity(3,3);

    Eigen::MatrixXf normals_eigen(vPlanes.size(),3);
    for (int P = 0; P < vPlanes.size(); P++)
        normals_eigen.block<1,3>(P,0) = vPlanes[P].coeffs.block<3,1>(0,0);

    for (int P = 0; P < vPlanes.size(); P++)
    {
        for (int Q = 0; Q < vPlanes.size(); Q++)
        {
            if ((vPlanes[P].type != vPlanes[Q].type) && !(vPlanes[P].type <= 1 && vPlanes[Q].type <= 1))
            {
                int N_inliers = 0;
                int N_planes = 0;
                //        total_points += vPlanes[P].cloud->points.size();
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

                for (int i = 0; i< vPlanes.size(); i++)
                {
                    if (min_ang(i,0) < ang_threshold)
                    {
                        N_inliers += vPlanes[i].cloud->points.size();
                        N_planes++;
                    }
                }

                if (N_inliers > N_inliers_max && N_planes > 1)
                {
                    N_inliers_max = N_inliers;
                    Eigen::MatrixXf current_eigDx(3,3);
                    current_eigDx << v1, v2, v3;
                    best_eigDx = current_eigDx;
                }
            }

        }
    }

    if (N_inliers_max > 0)
    {
        eigDx = sortManhattanDirections(best_eigDx);
        has_manhattan = true;
    }
}


Eigen::Matrix3f CurrentScene::sortManhattanDirections(Eigen::Matrix3f best_eigDx)
{
    Eigen::Vector3f v_floor(0.0f, -sin(M_PI/4), -sin(M_PI/4));
    Eigen::Vector3f v_front(0.0f, 0.0f, 1.0f);

    Eigen::MatrixXf angles_y, angles_z;
    angles_y = acos((v_floor.transpose()*best_eigDx).array().abs());
    angles_z = acos((v_front.transpose()*best_eigDx).array().abs());
    int index_x, index_y, index_z;

    if (angles_y(0,0) < angles_y(0,1) & angles_y(0,0) < angles_y(0,2))
        index_y = 0;
    else if (angles_y(0,1) < angles_y(0,0) & angles_y(0,1) < angles_y(0,2))
        index_y = 1;
    else
        index_y = 2;

    if (angles_z(0,0) < angles_z(0,1) & angles_z(0,0) < angles_z(0,2))
        index_z = 0;
    else if (angles_z(0,1) < angles_z(0,0) & angles_z(0,1) < angles_z(0,2))
        index_z = 1;
    else
        index_z = 2;

//    std::cout << "index_y " << index_y << " index_z " << index_z << std::endl;

    if (index_y == index_z)
    {
        if (angles_y(0,index_y) < angles_z(0,index_z))
        {
            if (index_y != 0)
                if (angles_z(0,0) < angles_z(0,1) || angles_z(0,0) < angles_z(0,2))
                    index_z = 0;
            if (index_y != 1)
                if (angles_z(0,1) < angles_z(0,0) || angles_z(0,1) < angles_z(0,2))
                    index_z = 1;
            if (index_y != 2)
                if (angles_z(0,2) < angles_z(0,0) || angles_z(0,2) < angles_z(0,1))
                    index_z = 2;
         }
        else
        {
            if (index_z != 0)
                if (angles_y(0,0) < angles_y(0,1) || angles_y(0,0) < angles_y(0,2))
                    index_y = 0;
            if (index_z != 1)
                if (angles_y(0,1) < angles_y(0,0) || angles_y(0,1) < angles_y(0,2))
                    index_y = 1;
            if (index_z != 2)
                if (angles_y(0,2) < angles_y(0,0) || angles_y(0,2) < angles_y(0,1))
                    index_y = 2;
        }
    }

    if (index_y == 0)
        if (index_z == 1)
            index_x = 2;
        else
            index_x = 1;
    else if (index_y == 1)
        if (index_z == 0)
            index_x = 2;
        else
            index_x = 0;
    else
        if (index_z == 0)
            index_x = 1;
        else
            index_x = 0;

    Eigen::Vector3f dx = best_eigDx.col(index_x);
    Eigen::Vector3f dy = best_eigDx.col(index_y);
    Eigen::Vector3f dz = best_eigDx.col(index_z);

    if (dy(1) > 0)//(acos(v_floor.transpose()*dy) > M_PI/2)
        dy = -dy;
    if (dz(2) < 0)//(acos(v_front.transpose()*dz) > M_PI/2)
        dz = -dz;
    dx = dy.cross(dz);

    Eigen::MatrixXf final_eigDx(3,3);
    final_eigDx << dx, dy, dz;

    return final_eigDx;
}


