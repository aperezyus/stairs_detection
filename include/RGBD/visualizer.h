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

class Viewer {
public:
    Viewer() : cloud_viewer_ ("Viewer")   {
        cloud_viewer_.setBackgroundColor (0, 0, 0); // black
        // cloud_viewer_.setBackgroundColor (255, 255, 255); // white
        //         cloud_viewer_.addCoordinateSystem (1.0);

        // Configuration so first view looks like an image
        cloud_viewer_.setPosition (0, 0);
        cloud_viewer_.setSize (640, 480);
        cloud_viewer_.setCameraClipDistances (0.0, 10.00);
        cloud_viewer_.setCameraPosition(	0,	0,	0,  	// pos
                                            0,	0,	1, // focal
                                            0,	-1,	0);	// view
        cloud_viewer_.setCameraFieldOfView(1);
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
    void getColorType(double &r, double &g, double &b, int type);
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
    void drawBox(pcl::PointXYZ centroid, double z_dir, double y_dir, double x_dir, Eigen::Matrix3f dir, int Q);
    void drawPerson(float person_height, Eigen::Vector4f floor_normal);
    void drawPerson(float person_height, Eigen::Vector4f floor_normal, Eigen::Affine3d pose);
    // Create the axis_points which will be moved in the viewer with the updated pose
    void createAxis ();
    // Rotates the axis_points and draws them in the viewer
    void drawAxis (Eigen::Affine3d& pose);
    void drawAxis (Eigen::Matrix3f pose);
    void drawPlaneAxis (Plane plane, int index);
    void drawPlaneNormal (Plane plane, double r, double g, double b, int index);


    pcl::visualization::PCLVisualizer cloud_viewer_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr axis_points;
    int iteration;

};

#endif
