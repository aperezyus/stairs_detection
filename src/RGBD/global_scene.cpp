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
//#include "custom_functions.h"
#include "RGBD/visualizer.h"

void GlobalScene::findFloor(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
  float z_dist = 2.0;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
	
	while (cloud_filtered->points.size() < 200)
	{
                z_dist+=0.1f;
		//~ std::cout << "Creando nube filtrada de suelo a " << z_dist << "m." << std::endl;
        pcl::PassThrough<pcl::PointXYZ> pass;
		pass.setInputCloud(cloud);
		pass.setFilterFieldName("z");
		pass.setFilterLimits(0.0, z_dist);
		pass.setFilterLimitsNegative (false);
		pass.filter(*cloud_filtered);
		if (z_dist > 5)
			break;
	}
	
	int tries = 20;
	
	while ((!initial_floor) and (tries > 0))
	{
//        std::cout << "Trying with a cloud of " << cloud_filtered->points.size() << " points" << std::endl;
//        Viewer viewer_aux;
//        viewer_aux.drawCloud(cloud_filtered,255.0,255.0,255.0,100);
//        viewer_aux.cloud_viewer_.spin();

        initial_floor = this->subtractInitialPlane(cloud_filtered, floor_normal);
		tries -= 1;
		if (tries == 0)
		{
            z_dist+=0.1f;
			if (z_dist > 5)
				break;
//            std::cout << "Creando nube filtrada de suelo a " << z_dist << "m." << std::endl;
			pcl::PassThrough<pcl::PointXYZ> pass;
			pass.setInputCloud(cloud);
			pass.setFilterFieldName("z");
			pass.setFilterLimits(0.0, z_dist);
			pass.filter(*cloud_filtered);
//                        tries = 20;
		}
	}
	// if (initial_floor)
	// 	std::cout << "Y está situado a " << z_dist << "m." << std::endl;
}

bool GlobalScene::subtractInitialPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, Eigen::Vector4f &normal)
{
    bool initial_plane = true;
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>());
	
	*cloud_filtered=*cloud;    
	
    if (cloud_filtered->points.size()>4)//3
    {
		// Create the segmentation object for the planar model and set all the parameters
		pcl::SACSegmentation<pcl::PointXYZ> seg;
		pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
		pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
//		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
		seg.setOptimizeCoefficients (true);
		seg.setModelType (pcl::SACMODEL_PLANE);
		seg.setMethodType (pcl::SAC_RANSAC);
		seg.setMaxIterations (100);
		seg.setDistanceThreshold (0.04);//0.01
	
		// Segment the largest planar component from the remaining cloud
		seg.setInputCloud (cloud_filtered);
		seg.segment (*inliers, *coefficients);
		
		//Asigna coeficientes a los parámetros del plano   Ax + By + Cz + D =0
    float A=(coefficients->values[0]);
    float B=(coefficients->values[1]);
    float C=(coefficients->values[2]);
    float D=(coefficients->values[3]);

    if (D < 0) {A = -A; B = -B; C = -C; D = -D;}

    Eigen::Vector3f v_plane(A,B,C);
    Eigen::Vector3f v_floor(0.0f, -sin(M_PI_4), -sin(M_PI_4));

    float angle = pcl::rad2deg(acos(v_plane.dot(v_floor)));
//    std::cout << angle << std::endl;
//    std::cout << D << std::endl;


    if (angle < 45 && fabs(D) > 1.0f && fabs(D) < 1.6f)
    {
//      std::cout << "Suelo encontrado" << std::endl;
//      std::cout << "Coeficientes de la ecuación del plano -> A: " << A << " B: " << B << " C: " << C << " D: " << D << std::endl;
//      std::cout << "Ángulo con la horizontal hacia abajo " << angle << std::endl;
//      std::cout << "El plano contiene " << inliers->indices.size() << " puntos" << std::endl;

      normal = Eigen::Vector4f(A,B,C,D);
    }
    else
      initial_plane = false;

		
//		// std::cout << "Coeficientes de la ecuación del plano -> A: " << A << " B: " << B << " C: " << C << " D: " << D << std::endl;

//		if ((B>0) and (C>0))
//		{
//			A = -A;
//			B = -B;
//			C = -C;
//			D = -D;
//		}

//		if ((B<0) and (C<0))
//		{
//			// ángulo con la horizontal en positivo hacia abajo
//			float angle = atan2f(B,C)*180/M_PI+180;
			
			
//			// std::cout << angle << std::endl;
//			// std::cout << D << std::endl;
//			// std::cout << fabs(A) << std::endl;
//			// if ((angle > 30) and (angle < 60) and (fabs(A)<0.20) and (D>1.2) and (D<1.6) and (inliers->indices.size() > 200))
//            if ((angle > 10) and (angle < 90) and (fabs(A)<0.20) and (D>1.0) and (D<1.5) and (inliers->indices.size() > 20))
//			{
//                 std::cout << "Suelo encontrado" << std::endl;
//                 std::cout << "Coeficientes de la ecuación del plano -> A: " << A << " B: " << B << " C: " << C << " D: " << D << std::endl;
//                 std::cout << "Ángulo con la horizontal hacia abajo " << angle << std::endl;
//                 std::cout << "El plano contiene " << inliers->indices.size() << " puntos" << std::endl;
				
//				normal = Eigen::Vector4f(A,B,C,D);
				
////                /* Nube de puntos del plano encontrado. Mostrar para debug */
////                floor.reset(new pcl::PointCloud<pcl::PointXYZ>);
////                pcl::copyPointCloud(*cloud_filtered,*inliers,*floor);
//			}
//			else
//				initial_plane = false;
//		}
//		else
//			initial_plane = false;
			
		
		if (!initial_plane)
		{
			pcl::ExtractIndices<pcl::PointXYZ> extract;
			extract.setInputCloud (cloud_filtered);
			extract.setIndices (inliers);
			extract.setNegative (true);
			extract.filter (*cloud);
		}
		// else
		// {
		// 	floor.reset(new pcl::PointCloud<pcl::PointXYZ>);
		// 	pcl::ExtractIndices<pcl::PointXYZ> extract;
		// 	extract.setInputCloud (cloud_filtered);
		// 	extract.setIndices (inliers);
		// 	extract.setNegative (false);
		// 	extract.filter (*floor);
		// }

	}
	else
		initial_plane=false;
		
	return (initial_plane);
}

void GlobalScene::computeCamera2FloorMatrix ()
{
	Eigen::Matrix3d R; 
    Eigen::Vector3d t = Eigen::Vector3d (0.f, 0.f, 0.f);  

    double a,b,c,d;
    
    c = sqrt(1/(1+floor_normal(2)*floor_normal(2)/(floor_normal(1)*floor_normal(1))));
    d = -c*floor_normal(2)/floor_normal(1);
    b = 1/(floor_normal(0)+(c*floor_normal(1)*(c*floor_normal(1)-d*floor_normal(2)))/floor_normal(0)-(d*floor_normal(2)*(c*floor_normal(1)-d*floor_normal(2)))/floor_normal(0));
    a = b*(c*floor_normal(1)-d*floor_normal(2))/floor_normal(0);
    
    //~ std::cout << a << b << c << d << std::endl;
    
    R << a, -b*c, b*d, b, a*c, -a*d, 0, d, c;
    t(1) = -a*floor_normal(3);

    c2f = ( Eigen::Translation3d (t) * Eigen::AngleAxisd (R));
    f2c = c2f.inverse();
    person_height = fabs(c2f(1,3));

    // std::cout << c2f.matrix() << std::endl;
}


void GlobalScene::computeCamera2AbsoluteMatrix(Eigen::Affine3d f2a)
{
  c2a = f2a*c2f;
  a2c = c2a.inverse();
}

void GlobalScene::computeCamera2AbsoluteMatrixWithOdometry(Eigen::Affine3d c2c0)
{
  c2a = c02a*c2c0;
  a2c = c2a.inverse();
}

void GlobalScene::computeIncrementalCamera2FloorMatrix ()
{
	Eigen::Affine3d old_c2f = c2f;
	
	Eigen::Matrix3d R; 
    Eigen::Vector3d t = Eigen::Vector3d (0.f, 0.f, 0.f);  
    
    double a,b,c,d;
    
    c = sqrt(1/(1+floor_normal(2)*floor_normal(2)/(floor_normal(1)*floor_normal(1))));
    d = -c*floor_normal(2)/floor_normal(1);
    b = 1/(floor_normal(0)+(c*floor_normal(1)*(c*floor_normal(1)-d*floor_normal(2)))/floor_normal(0)-(d*floor_normal(2)*(c*floor_normal(1)-d*floor_normal(2)))/floor_normal(0));
    a = b*(c*floor_normal(1)-d*floor_normal(2))/floor_normal(0);
    
    //~ std::cout << a << b << c << d << std::endl;
    
    R << a, -b*c, b*d, b, a*c, -a*d, 0, d, c;
	// t(1) = -a*floor_normal(3);
	t(1) = person_height;

    c2f = ( Eigen::Translation3d (t) * Eigen::AngleAxisd (R));
    f2c = c2f.inverse();
    
    
    Eigen::Matrix3d rot1,rot2,rot3;
    rot1 = old_c2f.rotation();
    rot2 = f2c.rotation();
    rot3 = rot1*rot2;
    f2f = Eigen::Translation3d(Eigen::Vector3d(0,0,0)) * Eigen::AngleAxisd(rot3);

    // std::cout << c2f.matrix() << std::endl;
    //~ f2f(0,3) = 0;
    //~ f2f(1,3) = 0;
    //~ f2f(2,3) = 0;
    //~ f2f(3,3) = 1;
    //~ f2f(3,0) = 0;
    //~ f2f(3,1) = 0;
    //~ f2f(3,2) = 0;
    //~ float x,y,z,roll,pitch,yaw;
    //~ pcl::getTranslationAndEulerAngles (f2f.cast<float>(), x, y, z, roll, pitch, yaw);
    //~ std::cout << x << y << z << pcl::rad2deg(roll) << pcl::rad2deg(pitch) << pcl::rad2deg(yaw) << std::endl;
    //~ f2f = f2f.inverse();
    
    //~ std::cout << f2f.matrix() << std::endl;
}



void GlobalScene::findFloorFast(std::vector<Plane> vPlanes)
{
    new_floor = false;
	for (int Q=0; Q<vPlanes.size();Q++)
	{
		
//		Eigen::Vector4f normal = Eigen::Vector4f(vPlanes[Q].coeffs[0],vPlanes[Q].coeffs[1],vPlanes[Q].coeffs[2], vPlanes[Q].coeffs[3]);

//    float angle = acos(vPlanes[Q].coeffs.head<3>().dot(floor_normal.head<3>()));
//		float angle;
//		angle = getAngle(normal, floor_normal);

        float dot = vPlanes[Q].coeffs.head<3>().dot(floor_normal.head<3>());
        dot = ( dot < -1.0f ? -1.0f : ( dot > 1.0f ? 1.0f : dot ) );

//        std::cout << "plane " << Q << " size " << vPlanes[Q].cloud->points.size() << " coeffs " << vPlanes[Q].coeffs << std::endl;
//        std::cout << vPlanes[Q].coeffs.head<3>().dot(floor_normal.head<3>()) << " " << acos(vPlanes[Q].coeffs.head<3>().dot(floor_normal.head<3>())) << " " << pcl::rad2deg(acos(vPlanes[Q].coeffs.head<3>().dot(floor_normal.head<3>()))) << std::endl;
//        std::cout << pcl::rad2deg(angle) << " " << (fabs(fabs(vPlanes[Q].coeffs[3])-fabs(floor_normal[3]))) << " " << vPlanes[Q].cloud->points.size() << std::endl;
        if ((pcl::rad2deg(acos(dot)) < 15) and (fabs(fabs(vPlanes[Q].coeffs[3])-fabs(floor_normal[3]))<=0.08f))
		// if (pcl::rad2deg(angle)<10)
		{
//            std::cout << "Se ha cambiado el suelo con un angulo de " << pcl::rad2deg(angle) << std::endl << std::endl;
            floor_normal = vPlanes[Q].coeffs;
            new_floor = true;
			current_floor = Q;
//			return floor_found;
            break;
		}
	}
//	return floor_found;
}

void GlobalScene::updateManhattanDirections(Eigen::Matrix3f manhattan_dirs)
{
    if (!has_manhattan)
    {
        eigDx = manhattan_dirs;
        has_manhattan = true;
    }
    else
    {

        float angle_z = acos(manhattan_dirs.col(2).dot(eigDx.col(2)));
        Eigen::Vector3f cross_z = manhattan_dirs.col(2).cross(eigDx.col(2));
        if (eigDx.col(1).dot(cross_z) < 0) { // Or > 0
          angle_z = -angle_z;
        }


//        float angle_z = asin((manhattan_dirs.col(2).cross(eigDx.col(2))).norm());
//        float angle_x = asin((manhattan_dirs.col(0).cross(eigDx.col(0))).norm());

//        std::cout << pcl::rad2deg(angle_z) << " " << pcl::rad2deg(angle_x) << std::endl;

        // if approx M_PI_2 turn in +y
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

void GlobalScene::updateFloorWithManhattan()
{
  floor_normal.head<3>() = eigDx.col(1);
//  computeCamera2FloorMatrix ();


//    f2c.matrix().block<3,3>(0,0) = eigDx.inverse().cast<double>();
//    c2f = f2c.inverse();

//  Eigen::Affine3d f2m = Eigen::Affine3d::Identity();
//  f2m.matrix().block<3,3>(0,0) = eigDx.cast<double>()* c2f.matrix().block<3,3>(0,0).inverse();

//  c2f = f2m*c2f;

//  c2f.matrix().block<3,1>(0,3) = eigDx.cast<double>() * c2f.inverse().matrix().block<3,3>(0,0) * c2f.matrix().block<3,1>(0,3);

  c2f.matrix().block<3,3>(0,0) = eigDx.cast<double>().inverse();
  f2c = c2f.inverse();

}


void GlobalScene::getManhattanDirections(CurrentScene &scene)
{
//    std::cout << c2f.matrix() << std::endl;


    if (new_floor)
    {
        std::cout << "getting Manhattan dirs from planes with floor" << std::endl;
        scene.getManhattanDirectionsFromPlanesWithFloor(f2c);
    }
    if (scene.has_manhattan == false)
    {
        std::cout << "getting Manhattan dirs from planes" << std::endl;
        scene.getManhattanDirectionsFromPlanes();
        if (scene.has_manhattan == false)
        {
                   std::cout << "getting Manhattan dirs from normals" << std::endl;
            scene.getManhattanDirectionsFromNormals();
        }
    }


  if (scene.has_manhattan)
  {

//      std::cout << eigDx << std::endl;

      updateManhattanDirections(scene.eigDx);
      updateFloorWithManhattan();

//      std::cout << c2f.matrix() << std::endl << std::endl;
//      std::cout << eigDx << std::endl << std::endl;
  }
}
