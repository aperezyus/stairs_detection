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

#include "RGBD/visualizer.h"
//#include "custom_functions.h"


// CLASS VISUALIZER
//~ class Viewer
//~ {
	//~ Viewer::Viewer() : cloud_viewer_ ("Viewer")
	//~ {
		//~ cloud_viewer_.setBackgroundColor (0, 0, 0); // black
		//cloud_viewer_.setBackgroundColor (255, 255, 255); // white
		//~ cloud_viewer_.addCoordinateSystem (1.0);
		//~ cloud_viewer_.initCameraParameters ();
		//~ cloud_viewer_.setPosition (0, 0);
		//~ cloud_viewer_.setSize (640, 480);
		//~ cloud_viewer_.setCameraClipDistances (0.01, 10.01);    
		//~ cloud_viewer_.setCameraClipDistances(0.75,9);
		//~ cloud_viewer_.setCameraPosition(0,1.90,-2.5,-0.075,0.2,1.2,0.0025,0.9,0.45);
		//~ cloud_viewer_.setCameraFieldOfView(0.57);
		//~ name_ = "camera";    
	//~ }

	void Viewer::drawCloud (pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, double r, double g, double b, int index)
	{
		char cloud_name[1024];
		sprintf (cloud_name, "cloud__%d", index);
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color(cloud,r,g,b);
		bool exists;
		exists = cloud_viewer_.updatePointCloud<pcl::PointXYZ> (cloud, cloud_color, cloud_name);
		if (!exists)
			cloud_viewer_.addPointCloud<pcl::PointXYZ> (cloud, cloud_color, cloud_name);
		cloud_viewer_.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 8, cloud_name);
	}

  void Viewer::drawCloud (pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, Eigen::Affine3d pose, double r, double g, double b, int index)
  {
    char cloud_name[1024];
    sprintf (cloud_name, "cloud__%d", index);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_to_draw (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*cloud,*cloud_to_draw,pose);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color(cloud_to_draw,r,g,b);
    bool exists;
    exists = cloud_viewer_.updatePointCloud<pcl::PointXYZ> (cloud_to_draw, cloud_color, cloud_name);
    if (!exists)
      cloud_viewer_.addPointCloud<pcl::PointXYZ> (cloud_to_draw, cloud_color, cloud_name);
    cloud_viewer_.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 8, cloud_name);
  }
	
  void Viewer::drawColorCloud (pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, int index)
	{
		char cloud_name[1024];
		sprintf (cloud_name, "color_cloud__%d", index);
		pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
		bool exists;
		exists = cloud_viewer_.updatePointCloud (cloud,rgb, cloud_name);
		if (!exists)
			cloud_viewer_.addPointCloud (cloud,rgb, cloud_name);

		cloud_viewer_.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, cloud_name);
	}

  void Viewer::drawColorCloud (pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, Eigen::Affine3d pose, int index)
  {
    char cloud_name[1024];
    sprintf (cloud_name, "color_cloud__%d", index);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_to_draw (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::transformPointCloud(*cloud,*cloud_to_draw,pose);

    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_to_draw);
    bool exists;
    exists = cloud_viewer_.updatePointCloud (cloud_to_draw,rgb, cloud_name);
    if (!exists)
      cloud_viewer_.addPointCloud (cloud_to_draw,rgb, cloud_name);

    cloud_viewer_.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, cloud_name);
  }

	void Viewer::drawSphere (pcl::PointXYZ point, double radius, double r, double g, double b, int index)
	{
		char sphere_name[1024];
		sprintf (sphere_name, "sphere__%d", index);
		cloud_viewer_.removeShape(sphere_name);
		cloud_viewer_.addSphere(point,radius,r,g,b,sphere_name);
	}
	
	void Viewer::drawNormals (pcl::PointCloud<pcl::Normal>::Ptr & normals, pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud)
	{
		cloud_viewer_.removeShape("normals");
        cloud_viewer_.addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (cloud, normals, 1, 0.05f, "normals");
	}
	
	void Viewer::drawRectangle (pcl::PointCloud<pcl::PointXYZ>::Ptr vertices, double r, double g, double b, std::string string)
	{
		std::stringstream ss;
		ss << "rectangle_" << string;
		const std::string tmp = ss.str();
		const char* name = tmp.c_str();

		pcl::PolygonMesh polygon;

		pcl::Vertices vertex;
		for (int P = 0; P< vertices->points.size(); P++)
		{
			vertex.vertices.push_back(P);
		}

		std::vector<pcl::Vertices> vvertex;
		vvertex.push_back(vertex);

		sensor_msgs::PointCloud2 msg;
		pcl::PCLPointCloud2 msg_aux;
		pcl::toROSMsg( *vertices, msg );
		pcl_conversions::toPCL( msg, msg_aux );

		polygon.cloud = msg_aux;
		polygon.polygons = vvertex;

		cloud_viewer_.addPolygonMesh (polygon,name);
		//~ cloud_viewer_.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color(0),color(1),color(2), name,v2);
		cloud_viewer_.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, r,g,b, name);
		// cloud_viewer_.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY,0.40f,name);
		cloud_viewer_.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY,0.7f,name);
		// cloud_viewer_.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY,1,name);
	}

	void Viewer::drawPlanesRandom (std::vector<Plane> vPlanes)
	{
                for (int Q=0; Q<vPlanes.size(); Q++)
		{
			int random_r = rand() % 255;
			int random_g = rand() % 255;
			int random_b = rand() % 255;
			this->drawCloud(vPlanes[Q].cloud, (float) random_r, (float) random_g, (float) random_b, Q);
			// this->drawCloud(vPlanes[Q].cloud, 0, 255.0,  0, Q);

			// if (Q == 0)
			// 	this->drawCloud(vPlanes[Q].cloud, 0.0f, 255.0f, 0.0f, Q);
			// else if (Q == 1)
			// 	this->drawCloud(vPlanes[Q].cloud, 0.0f, 0.0f, 255.0f, Q);
			// else if (Q == 2)
			// 	this->drawCloud(vPlanes[Q].cloud, 255.0f, 255.0f, 0.0f, Q);
			// else if (Q == 3)
			// 	this->drawCloud(vPlanes[Q].cloud, 255.0f, 100.0f, 0.0f, Q);
			// else if (Q == 4)
			// 	this->drawCloud(vPlanes[Q].cloud, 0.0f, 100.0f, 255.0f, Q);
			// else if (Q == 5)
			// 	this->drawCloud(vPlanes[Q].cloud, 255.0f, 0.0f, 0.0f, Q);

		}
	}		

  void Viewer::drawCloudsRandom (std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > vClouds)
  {
          for (int Q=0; Q<vClouds.size(); Q++)
          {
//                        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_to_draw (new pcl::PointCloud<pcl::PointXYZ>);
//                        pcl::transformPointCloud(*vClouds[Q],*cloud_to_draw,pose);

                  int random_r = rand() % 255;
                  int random_g = rand() % 255;
                  int random_b = rand() % 255;
                  this->drawCloud(vClouds[Q], (float) random_r, (float) random_g, (float) random_b, Q+100);
                  // this->drawCloud(cloud_to_draw, 0, 255.0,  0, Q);

                  // if (Q == 0)
                  // 	this->drawCloud(vPlanes[Q].cloud, 0.0f, 255.0f, 0.0f, Q);
                  // else if (Q == 1)
                  // 	this->drawCloud(vPlanes[Q].cloud, 0.0f, 0.0f, 255.0f, Q);
                  // else if (Q == 2)
                  // 	this->drawCloud(vPlanes[Q].cloud, 255.0f, 255.0f, 0.0f, Q);
                  // else if (Q == 3)
                  // 	this->drawCloud(vPlanes[Q].cloud, 255.0f, 100.0f, 0.0f, Q);
                  // else if (Q == 4)
                  // 	this->drawCloud(vPlanes[Q].cloud, 0.0f, 100.0f, 255.0f, Q);
                  // else if (Q == 5)
                  // 	this->drawCloud(vPlanes[Q].cloud, 255.0f, 0.0f, 0.0f, Q);

          }
  }

  void Viewer::drawCloudsRandom (std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > vClouds, Eigen::Affine3d pose)
  {
          for (int Q=0; Q<vClouds.size(); Q++)
          {
                        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_to_draw (new pcl::PointCloud<pcl::PointXYZ>);
                        pcl::transformPointCloud(*vClouds[Q],*cloud_to_draw,pose);

                  int random_r = rand() % 255;
                  int random_g = rand() % 255;
                  int random_b = rand() % 255;
                  this->drawCloud(cloud_to_draw, (float) random_r, (float) random_g, (float) random_b, Q+100);
          }
  }


	void Viewer::drawPlanesRandom (std::vector<Plane> vPlanes, Eigen::Affine3d pose)
	{
		for (int Q=0; Q<vPlanes.size(); Q++)
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_to_draw (new pcl::PointCloud<pcl::PointXYZ>);
			pcl::transformPointCloud(*vPlanes[Q].cloud,*cloud_to_draw,pose);

			int random_r = rand() % 255;
			int random_g = rand() % 255;
			int random_b = rand() % 255;
			this->drawCloud(vPlanes[Q].cloud, (float) random_r, (float) random_g, (float) random_b, Q);
			// this->drawCloud(cloud_to_draw, 0, 255.0,  0, Q);

			// if (Q == 0)
			// 	this->drawCloud(vPlanes[Q].cloud, 0.0f, 255.0f, 0.0f, Q);
			// else if (Q == 1)
			// 	this->drawCloud(vPlanes[Q].cloud, 0.0f, 0.0f, 255.0f, Q);
			// else if (Q == 2)
			// 	this->drawCloud(vPlanes[Q].cloud, 255.0f, 255.0f, 0.0f, Q);
			// else if (Q == 3)
			// 	this->drawCloud(vPlanes[Q].cloud, 255.0f, 100.0f, 0.0f, Q);
			// else if (Q == 4)
			// 	this->drawCloud(vPlanes[Q].cloud, 0.0f, 100.0f, 255.0f, Q);
			// else if (Q == 5)
			// 	this->drawCloud(vPlanes[Q].cloud, 255.0f, 0.0f, 0.0f, Q);

		}
	}		


	void Viewer::drawPlaneTypes (std::vector<Plane> vPlanes)
	{
		for (int Q=0; Q<vPlanes.size(); Q++)
		{
			// std::cout << vPlanes[Q].type << std::endl;
			if (vPlanes[Q].type == 0)
				this->drawCloud(vPlanes[Q].cloud, 0.0f, 255.0f, 0.0f, Q);
                        else if (vPlanes[Q].type == 1)
                                this->drawCloud(vPlanes[Q].cloud, 0.0f, 0.0f, 255.0f, Q);
                        else if (vPlanes[Q].type == 2)
                                this->drawCloud(vPlanes[Q].cloud, 255.0f, 255.0f, 0.0f, Q);
                        else if (vPlanes[Q].type == 3)
                                this->drawCloud(vPlanes[Q].cloud, 255.0f, 255.0f, 0.0f, Q);
                        else if (vPlanes[Q].type == 4)
                                this->drawCloud(vPlanes[Q].cloud, 255.0f, 255.0f, 0.0f, Q);
                        else if (vPlanes[Q].type == 5)
                                this->drawCloud(vPlanes[Q].cloud, 255.0f, 0.0f, 0.0f, Q);
		}
	}	
	
	void Viewer::drawPlaneTypesContour (std::vector<Plane> vPlanes)
	{
		for (int Q=0; Q<vPlanes.size(); Q++)
		{
			if (vPlanes[Q].contour == NULL)
				vPlanes[Q].getContour();

			if (vPlanes[Q].type == 0)
				this->drawCloud(vPlanes[Q].contour, 0.0f, 255.0f, 0.0f, Q);
			else if (vPlanes[Q].type == 1)
				this->drawCloud(vPlanes[Q].contour, 0.0f, 0.0f, 255.0f, Q);
			else if (vPlanes[Q].type == 2)
				this->drawCloud(vPlanes[Q].contour, 255.0f, 255.0f, 0.0f, Q);
			else if (vPlanes[Q].type == 3)
				this->drawCloud(vPlanes[Q].contour, 255.0f, 255.0f, 0.0f, Q);
			else if (vPlanes[Q].type == 4)
				this->drawCloud(vPlanes[Q].contour, 255.0f, 255.0f, 0.0f, Q);
			else if (vPlanes[Q].type == 5)
				this->drawCloud(vPlanes[Q].contour, 255.0f, 0.0f, 0.0f, Q);
		}
	}	
  void Viewer::drawPlaneTypesContour (std::vector<Plane> vPlanes, Eigen::Affine3d pose)
  {
    for (int Q=0; Q<vPlanes.size(); Q++)
    {
      if (vPlanes[Q].contour == NULL)
        vPlanes[Q].getContour();

      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_to_draw (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::transformPointCloud(*vPlanes[Q].contour,*cloud_to_draw,pose);


      if (vPlanes[Q].type == 0)
        this->drawCloud(cloud_to_draw, 0.0f, 255.0f, 0.0f, Q);
      else if (vPlanes[Q].type == 1)
        this->drawCloud(cloud_to_draw, 0.0f, 0.0f, 255.0f, Q);
      else if (vPlanes[Q].type == 2)
        this->drawCloud(cloud_to_draw, 255.0f, 255.0f, 0.0f, Q);
      else if (vPlanes[Q].type == 3)
        this->drawCloud(cloud_to_draw, 255.0f, 255.0f, 0.0f, Q);
      else if (vPlanes[Q].type == 4)
        this->drawCloud(cloud_to_draw, 255.0f, 255.0f, 0.0f, Q);
      else if (vPlanes[Q].type == 5)
        this->drawCloud(cloud_to_draw, 255.0f, 0.0f, 0.0f, Q);
    }
  }

	void Viewer::drawPlaneTypes (std::vector<Plane> vPlanes, Eigen::Affine3d pose)
	{
		for (int Q=0; Q<vPlanes.size(); Q++)
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_to_draw (new pcl::PointCloud<pcl::PointXYZ>);
			pcl::transformPointCloud(*vPlanes[Q].cloud,*cloud_to_draw,pose);

			if (vPlanes[Q].type == 0)
				this->drawCloud(cloud_to_draw, 0.0f, 255.0f, 0.0f, Q);
			else if (vPlanes[Q].type == 1)
				this->drawCloud(cloud_to_draw, 0.0f, 0.0f, 255.0f, Q);
			else if (vPlanes[Q].type == 2)
				this->drawCloud(cloud_to_draw, 255.0f, 255.0f, 0.0f, Q);
			else if (vPlanes[Q].type == 3)
				this->drawCloud(cloud_to_draw, 255.0f, 255.0f, 0.0f, Q);
			else if (vPlanes[Q].type == 4)
				this->drawCloud(cloud_to_draw, 255.0f, 255.0f, 0.0f, Q);
			else if (vPlanes[Q].type == 5)
				this->drawCloud(cloud_to_draw, 255.0f, 0.0f, 0.0f, Q);
		}
	}
	
	void Viewer::drawBox(pcl::PointXYZ centroid, float z_dir, float y_dir, float x_dir, Eigen::Matrix3f dir, int Q)
	{
		//~ Eigen::Vector4f vector_centroid = Vector4f(centroid.x,centroid.y,centroid.z,1);
		const Eigen::Quaternionf qfinal(dir);
		const Eigen::Vector3f tfinal = Eigen::Vector3f(centroid.x, centroid.y, centroid.z);
		char cube_name[1024];
		sprintf (cube_name, "Cube_%d", unsigned(Q));

		cloud_viewer_.addCube(tfinal, qfinal, z_dir, y_dir, x_dir,cube_name); 
	}
	
	void Viewer::drawPerson(float person_height, Eigen::Vector4f floor_normal)
	{
		cloud_viewer_.removeShape("head");
		cloud_viewer_.removeShape("body");
		
		Eigen::Vector3f perpendicular(floor_normal(0),-floor_normal(2),floor_normal(1));
		
		pcl::ModelCoefficients cylinder_coeff;
		cylinder_coeff.values.resize (7);    // We need 7 values
		cylinder_coeff.values[0] = 0.30f*floor_normal(0)+0.10f*perpendicular(0);
		cylinder_coeff.values[1] = 0.30f*floor_normal(1)+0.10f*perpendicular(1);
		cylinder_coeff.values[2] = 0.30f*floor_normal(2)+0.10f*perpendicular(2);
		cylinder_coeff.values[3] = -floor_normal(0)*(person_height-0.00f);
		cylinder_coeff.values[4] = -floor_normal(1)*(person_height-0.00f);
		cylinder_coeff.values[5] = -floor_normal(2)*(person_height-0.00f);
		cylinder_coeff.values[6] = 0.10f;
		cloud_viewer_.addCylinder (cylinder_coeff,"body");
		
		cloud_viewer_.addSphere(pcl::PointXYZ(	0.5f*floor_normal(0)+0.10f*perpendicular(0),
												0.5f*floor_normal(1)+0.10f*perpendicular(1),
												0.5f*floor_normal(2)+0.10f*perpendicular(2)),
												0.25f,"head");
	}

	void Viewer::drawPerson(float person_height, Eigen::Vector4f floor_normal, Eigen::Affine3d pose)
	{
		cloud_viewer_.removeShape("head");
		cloud_viewer_.removeShape("body");
		
		Eigen::Vector3f perpendicular(floor_normal(0),-floor_normal(2),floor_normal(1));

		pcl::ModelCoefficients cylinder_coeff;
		cylinder_coeff.values.resize (7);    // We need 7 values
		cylinder_coeff.values[0] = pose.translation()[0] + 0.30f*floor_normal(0)+0.10f*perpendicular(0);
		cylinder_coeff.values[1] = pose.translation()[1] + 0.30f*floor_normal(1)+0.10f*perpendicular(1);
		cylinder_coeff.values[2] = pose.translation()[2] + 0.30f*floor_normal(2)+0.10f*perpendicular(2);
		cylinder_coeff.values[3] = -floor_normal(0)*(person_height-0.00f);
		cylinder_coeff.values[4] = -floor_normal(1)*(person_height-0.00f);
		cylinder_coeff.values[5] = -floor_normal(2)*(person_height-0.00f);
		cylinder_coeff.values[6] = 0.10f;
		cloud_viewer_.addCylinder (cylinder_coeff,"body");
		
		cloud_viewer_.addSphere(pcl::PointXYZ(	pose.translation()[0] + 0.5f*floor_normal(0)+0.10f*perpendicular(0),
												pose.translation()[1] + 0.5f*floor_normal(1)+0.10f*perpendicular(1),
												pose.translation()[2] + 0.5f*floor_normal(2)+0.10f*perpendicular(2)),
												0.25f,"head");
	}
	
	void Viewer::drawAxis (Eigen::Affine3d& pose)
	{
		//~ std::cout << pose.matrix() << std::endl;
		pcl::PointCloud<pcl::PointXYZ>::Ptr axis_points_aux (new pcl::PointCloud<pcl::PointXYZ>);
//		transformPointCloudCustom(*axis_points,*axis_points_aux,pose);
        pcl::transformPointCloud(*axis_points,*axis_points_aux,pose);
		
		*axis_points = *axis_points_aux;
		
		
		char x_arrow[1024];
		sprintf (x_arrow, "x_arrow_%d", iteration);
		char y_arrow[1024];
		sprintf (y_arrow, "y_arrow_%d", iteration);
		char z_arrow[1024];
		sprintf (z_arrow, "z_arrow_%d", iteration);
		
		if (iteration == 0)
		{
			cloud_viewer_.removeShape("x_arrow_0");
			cloud_viewer_.removeShape("y_arrow_0");
			cloud_viewer_.removeShape("z_arrow_0");
		}
		
		if ((iteration == 0) or (iteration % 2 == 0))
		{
			cloud_viewer_.addArrow (axis_points->points[1], axis_points->points[0], 1.0f, 0.0f, 0.0f, false, x_arrow);
			cloud_viewer_.addArrow (axis_points->points[2], axis_points->points[0], 0.0f, 1.0f, 0.0f, false, y_arrow);
			cloud_viewer_.addArrow (axis_points->points[3], axis_points->points[0], 0.0f, 0.0f, 1.0f, false, z_arrow);
		}
	}	

	void Viewer::drawAxis (Eigen::Matrix3f pose)
	{
		//~ std::cout << pose.matrix() << std::endl;
    Eigen::Affine3d pose_affine = Eigen::Translation3d(Eigen::Vector3d(0,0,0)) * Eigen::AngleAxisd(pose.cast<double>());

		pcl::PointCloud<pcl::PointXYZ>::Ptr axis_points_aux (new pcl::PointCloud<pcl::PointXYZ>);
//		transformPointCloudCustom(*axis_points,*axis_points_aux,pose_affine);
        pcl::transformPointCloud(*axis_points,*axis_points_aux,pose_affine);
		
		*axis_points = *axis_points_aux;
		
		
		char x_arrow[1024];
		sprintf (x_arrow, "x_arrow_%d", iteration);
		char y_arrow[1024];
		sprintf (y_arrow, "y_arrow_%d", iteration);
		char z_arrow[1024];
		sprintf (z_arrow, "z_arrow_%d", iteration);
		
		if (iteration == 0)
		{
			cloud_viewer_.removeShape("x_arrow_0");
			cloud_viewer_.removeShape("y_arrow_0");
			cloud_viewer_.removeShape("z_arrow_0");
		}
		
		if ((iteration == 0) or (iteration % 2 == 0))
		{
			cloud_viewer_.addArrow (axis_points->points[1], axis_points->points[0], 1.0f, 0.0f, 0.0f, false, x_arrow);
			cloud_viewer_.addArrow (axis_points->points[2], axis_points->points[0], 0.0f, 1.0f, 0.0f, false, y_arrow);
			cloud_viewer_.addArrow (axis_points->points[3], axis_points->points[0], 0.0f, 0.0f, 1.0f, false, z_arrow);
		}
	}


        void Viewer::drawPlaneAxis (Plane plane, int index)
        {
            pcl::PointXYZ origin = plane.center;

            Eigen::Vector3f vec_x = origin.getVector3fMap() + plane.eigDx.col(0) * 0.5;
            pcl::PointXYZ point_x(vec_x(0),vec_x(1),vec_x(2));
            Eigen::Vector3f vec_y = origin.getVector3fMap() + plane.eigDx.col(1) * 0.5;
            pcl::PointXYZ point_y(vec_y(0),vec_y(1),vec_y(2));
            Eigen::Vector3f vec_z = origin.getVector3fMap() + plane.eigDx.col(2) * 0.5;
            pcl::PointXYZ point_z(vec_z(0),vec_z(1),vec_z(2));

            char x_arrow[1024];
            sprintf (x_arrow, "x_arrow_%d", index);
            char y_arrow[1024];
            sprintf (y_arrow, "y_arrow_%d", index);
            char z_arrow[1024];
            sprintf (z_arrow, "z_arrow_%d", index);

            cloud_viewer_.addArrow (point_x, origin, 1.0f, 0.0f, 0.0f, false, x_arrow);
            cloud_viewer_.addArrow (point_y, origin, 0.0f, 1.0f, 0.0f, false, y_arrow);
            cloud_viewer_.addArrow (point_z, origin, 0.0f, 0.0f, 1.0f, false, z_arrow);
        }
	
	void Viewer::createAxis ()
	{
		pcl::PointXYZ pto(0,0,0);
		pcl::PointXYZ ptx(1,0,0);
		pcl::PointXYZ pty(0,1,0);
		pcl::PointXYZ ptz(0,0,1);
		axis_points.reset(new pcl::PointCloud<pcl::PointXYZ>);
		axis_points->points.push_back(pto);
		axis_points->points.push_back(ptx);
		axis_points->points.push_back(pty);
		axis_points->points.push_back(ptz);
	}
	
	
	
  
  inline void 
  Viewer::drawCamera (const Eigen::Affine3f& pose, double r, double g, double b)
  {
    double focal = 575;
    double height = 480;
    double width = 640;
    
    // create a 5-point visual for each camera
    pcl::PointXYZ p1, p2, p3, p4, p5;
    p1.x=0; p1.y=0; p1.z=0;
    double angleX = RAD2DEG (2.0 * atan (width / (2.0*focal)));
    double angleY = RAD2DEG (2.0 * atan (height / (2.0*focal)));
    double dist = 0.75;
    double minX, minY, maxX, maxY;
    maxX = dist*tan (atan (width / (2.0*focal)));
    minX = -maxX;
    maxY = dist*tan (atan (height / (2.0*focal)));
    minY = -maxY;
    p2.x=minX; p2.y=minY; p2.z=dist;
    p3.x=maxX; p3.y=minY; p3.z=dist;
    p4.x=maxX; p4.y=maxY; p4.z=dist;
    p5.x=minX; p5.y=maxY; p5.z=dist;
    p1=pcl::transformPoint (p1, pose);
    p2=pcl::transformPoint (p2, pose);
    p3=pcl::transformPoint (p3, pose);
    p4=pcl::transformPoint (p4, pose);
    p5=pcl::transformPoint (p5, pose);
    std::stringstream ss;
    ss.str ("");
    ss << name_ << "_line1";
    cloud_viewer_.addLine (p1, p2, r, g, b, ss.str ());
    ss.str ("");
    ss << name_ << "_line2";
    cloud_viewer_.addLine (p1, p3, r, g, b, ss.str ());
    ss.str ("");
    ss << name_ << "_line3";
    cloud_viewer_.addLine (p1, p4, r, g, b, ss.str ());
    ss.str ("");
    ss << name_ << "_line4";
    cloud_viewer_.addLine (p1, p5, r, g, b, ss.str ());
    ss.str ("");
    ss << name_ << "_line5";
    cloud_viewer_.addLine (p2, p5, r, g, b, ss.str ());
    ss.str ("");
    ss << name_ << "_line6";
    cloud_viewer_.addLine (p5, p4, r, g, b, ss.str ());
    ss.str ("");
    ss << name_ << "_line7";
    cloud_viewer_.addLine (p4, p3, r, g, b, ss.str ());
    ss.str ("");
    ss << name_ << "_line8";
    cloud_viewer_.addLine (p3, p2, r, g, b, ss.str ());    
  }
  
  inline void 
  Viewer::removeCamera ()
  {
    cloud_viewer_.removeShape (name_);
    std::stringstream ss;
    ss.str ("");
    ss << name_ << "_line1";
    cloud_viewer_.removeShape (ss.str ());
    ss.str ("");
    ss << name_ << "_line2";
    cloud_viewer_.removeShape (ss.str ());
    ss.str ("");
    ss << name_ << "_line3";
    cloud_viewer_.removeShape (ss.str ());
    ss.str ("");
    ss << name_ << "_line4";
    cloud_viewer_.removeShape (ss.str ());
    ss.str ("");
    ss << name_ << "_line5";
    cloud_viewer_.removeShape (ss.str ());
    ss.str ("");
    ss << name_ << "_line6";
    cloud_viewer_.removeShape (ss.str ());
    ss.str ("");
    ss << name_ << "_line7";//viewer.drawPlanesRandom(scene.vPlanes);
    cloud_viewer_.removeShape (ss.str ());
    ss.str ("");
    ss << name_ << "_line8";
    cloud_viewer_.removeShape (ss.str ());
  }
  
  void 
  Viewer::updateCamera(const Eigen::Affine3f& last_pose_estimate)
  {
    removeCamera();    
    drawCamera(last_pose_estimate, 1.0, 0.0, 0.0);      
  }

  void
  Viewer::show (const Eigen::Affine3f& new_viewer_pose)
  {
    viewer_pose_ = new_viewer_pose; 
  }


	//~ Eigen::Affine3f viewer_pose_;
	//~ std::string name_;
	//~ pcl::visualization::PCLVisualizer cloud_viewer_;
	//~ pcl::PointCloud<pcl::PointXYZ> axis_points;
//~ public:
  //~ EIGEN_MAKE_ALIGNED_OPERATOR_NEW

//~ };






