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

#ifndef VISUALIZER_H
#define VISUALIZER_H

#include <boost/filesystem.hpp>
#include <pcl/PCLHeader.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>

 #include <pcl/visualization/cloud_viewer.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include "RGBD/plane.h"
//#include "stair/stair_classes.h"

// CLASS VISUALIZER
struct Viewer
{
	// Constructor with standard settings
	Viewer() : cloud_viewer_ ("Viewer")
	{
		// // Values from
		// 0.718805,8.73687/				camera clip
		// -0.0751458,0.206077,1.13894/		camera pos
		// 0.0238492,1.90784,-2.41265/		camera focal
		// 0.00252639,0.901789,0.432168/	camera view
		// 0.523599/						fov
		// 640,480/							window size
		// 66,52 							window pos

                 cloud_viewer_.setBackgroundColor (0, 0, 0); // black
//		cloud_viewer_.setBackgroundColor (255, 255, 255); // white
//                cloud_viewer_.addCoordinateSystem (1.0);
		// cloud_viewer_.initCameraParameters ();
		cloud_viewer_.setPosition (0, 0);
		
		// cloud_viewer_.setSize (640, 480);
		// cloud_viewer_.setCameraClipDistances (0.01, 10.01);    
		// // cloud_viewer_.setCameraClipDistances(0.75,9);
		// cloud_viewer_.setCameraPosition(	0,		1.90,	-2.5, 	// focal
		// 									-0.075,	0.2,	1.2,  	// pos
		// 									0.0025,	0.9,	0.45);	// view
		// cloud_viewer_.setCameraFieldOfView(0.57);	
			
		cloud_viewer_.setSize (640, 480);
		cloud_viewer_.setCameraClipDistances (0.0, 10.00);    
		// cloud_viewer_.setCameraClipDistances(0.75,9);
		cloud_viewer_.setCameraPosition(	0,	0,	0,  	// pos
											0,	0,	1, // focal
											0,	-1,	0);	// view
		cloud_viewer_.setCameraFieldOfView(1);




		name_ = "camera";    
		
		// cloud_viewer_.addText ("No ascending stairs", 50, 50, 20, 1.0f, 1.0f, 1.0f, "uptext");
		// cloud_viewer_.addText ("No descending stairs", 50, 100, 20, 1.0f, 1.0f, 1.0f, "downtext");
		// cloud_viewer_.addText ("On the ground", 50, 150, 20, 1.0f, 1.0f, 1.0f, "infotext");
		
		iteration = 0;
	}

	// Adds a cloud to the viewer
	void drawCloud (pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, double r, double g, double b, int index);
  void drawCloud (pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, Eigen::Affine3d pose, double r, double g, double b, int index);
	void drawColorCloud (pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, int index);
  void drawColorCloud (pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, Eigen::Affine3d pose, int index);
	void drawSphere (pcl::PointXYZ point, double radius, double r, double g, double b, int index);
	void drawNormals (pcl::PointCloud<pcl::Normal>::Ptr & normals, pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud);
  void drawPlanesRandom (std::vector<Plane> vPlanes);
  void drawCloudsRandom (std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > vClouds);
  void drawCloudsRandom (std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > vClouds, Eigen::Affine3d pose);
  void drawPlanesRandom (std::vector<Plane> vPlanes, Eigen::Affine3d pose);
	void drawPlaneTypes (std::vector<Plane> vPlanes);
	void drawPlaneTypesContour (std::vector<Plane> vPlanes);
	void drawPlaneTypes (std::vector<Plane> vPlanes, Eigen::Affine3d pose);
  void drawPlaneTypesContour (std::vector<Plane> vPlanes, Eigen::Affine3d pose);
	void drawRectangle (pcl::PointCloud<pcl::PointXYZ>::Ptr vertices, double r, double g, double b, std::string string);

	// DRAWBOX
	/* z_dir, y_dir, x_dir are the lenghts in the direction of the colums of the dir matrix (3,2,1).
	 * For the stair_dir, we chose the x_dir to be in the length, but it changes with other dir matrices.
	 * 
	 */
	void drawBox(pcl::PointXYZ centroid, float z_dir, float y_dir, float x_dir, Eigen::Matrix3f dir, int Q);
	void drawPerson(float person_height, Eigen::Vector4f floor_normal);
	void drawPerson(float person_height, Eigen::Vector4f floor_normal, Eigen::Affine3d pose);
	// Create the axis_points which will be moved in the viewer with the updated pose
	void createAxis ();
	// Rotates the axis_points and draws them in the viewer
	void drawAxis (Eigen::Affine3d& pose);
	void drawAxis (Eigen::Matrix3f pose);
        void drawPlaneAxis (Plane plane, int index);
	
	// Former visOdo camera drawing functions
	inline void 
	drawCamera (const Eigen::Affine3f& pose, double r, double g, double b);  
	inline void 
	removeCamera ();  
	void 
	updateCamera(const Eigen::Affine3f& last_pose_estimate);
	void
	show (const Eigen::Affine3f& new_viewer_pose);


	Eigen::Affine3f viewer_pose_;
	std::string name_;
	pcl::visualization::PCLVisualizer cloud_viewer_;
	pcl::PointCloud<pcl::PointXYZ>::Ptr axis_points;
	int iteration;
	public:
	  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif
