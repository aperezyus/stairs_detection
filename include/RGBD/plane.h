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

#ifndef PLANE_H
#define PLANE_H

#include <boost/filesystem.hpp>
#include <pcl/PCLHeader.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common_headers.h>

#include <pcl/surface/concave_hull.h>
//#include <pcl/surface/convex_hull.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/ModelCoefficients.h>
//#include <pcl/filters/voxel_grid.h>


class Plane {
  public:
    Plane()	{
        main_dir.setZero();
        cloud.reset(new pcl::PointCloud<pcl::PointXYZ>() );

    }

    Plane(pcl::PointCloud<pcl::PointXYZ>::Ptr region, Eigen::Vector4f plane_vector) {
        coeffs = plane_vector;
        cloud.reset(new pcl::PointCloud<pcl::PointXYZ>() );
        *cloud = *region;
        main_dir.setZero();
    }

    Plane(pcl::PointCloud<pcl::PointXYZ>::Ptr region, Eigen::Vector4f plane_vector, pcl::PointXYZ centroid_in, pcl::PointXYZ centroid2f_in) {
        coeffs = plane_vector;
        cloud.reset(new pcl::PointCloud<pcl::PointXYZ>() );
        *cloud = *region;
        main_dir.setZero();
        centroid = centroid_in;
        centroid2f = centroid2f_in;
    }

    //// Computes centroid from cloud
    /// in: cloud
    /// out: (in class) centroid
    void getCentroid();

    //// Transforms centroid to floor coordinates
    /// in: c2f (transformation from camera frame to floor frame)
    /// out: (in class) centroid2f
    void getCentroid2Floor(Eigen::Affine3d c2f);

    //// Transform plane coeffs to floor coordinates
    /// in: c2f (transformation from camera frame to floor frame)
    /// out: (in class) coeffs2f
    void getCoeffs2Floor(Eigen::Affine3d c2f);

    //// Transforms cloud to floor coordinates
    /// in: c2f (transformation from camera frame to floor frame)
    /// out: (in class) cloud2f
    void getCloud2Floor(Eigen::Affine3d c2f);

    //// Computes contour (i.e. concave hull) of cloud
    /// out: (in class) contour
    void getContour();

    //// Transforms contour to floor coordinates
    /// in: c2f (transformation from camera frame to floor frame)
    /// out: (in class) contour2f
    void getContour2Floor(Eigen::Affine3d c2f);

    //// Computes principal directions of plane cloud
    /// out: (in class) main_dir (matrix containing by column the three principal directions)
    void getPrincipalDirections();

    //// Compute measurements of bounding rectangle (using main_dir from plane)
    /// out: (in class) width, length, height, center
    void getMeasurements();

    //// Compute measurements of bounding rectangle given rotation matrix
    /// in: c2m (rotation matrix to compute measurements)
    /// out: (in class) width, length, height, center
    void getMeasurements(Eigen::Matrix3f c2m);

    //// Compute measurements of bounding rectangle given transformation matrix
    /// in: p2w (transformation matrix to compute measurements)
    /// out: (in class) width, length, height, center
    void getMeasurements(Eigen::Affine3d & p2w);

    //// Compute measurements of bounding rectangle given rotation matrix and cloud to compute (other than plane.cloud)
    /// in: c2m (rotation matrix to compute measurements), custom_cloud (alternative cloud)
    /// out: (in class) width, length, height, center
    void getMeasurements(Eigen::Matrix3f c2m, pcl::PointCloud<pcl::PointXYZ>::Ptr custom_cloud);

    //// Get four vertices and center of bounding rectangle given following shape from main dirs of the plane:
    /// 0--------------1
    /// |       y      |
    /// |       |      |
    /// |    z--4      |
    /// |              |
    /// 3--------------2
    /// out: (in class) vertices
    void getVertices();

    //// Get four vertices and center of bounding rectangle given following shape, if Manhattan:
    /// 0--------------1
    /// |       z      |
    /// |       |      |
    /// |    x--4      |
    /// |              |
    /// 3--------------2
    /// in: c2m (directions from camera to other reference frame, e.g. Manhattan)
    /// out: (in class) vertices
    void getVertices(Eigen::Matrix3f c2m);

    //// Compute area of contour (i.e. concave hull)
    /// in: contour
    /// return: contour_area (in m^2)
    float getContourArea();

    //// Compute area of bounding rectangle
    /// in: width, length
    /// return: rectangle_area (in m^2)
    float getRectangleArea();

    //// Compute area of bounding rectangle given specific directions
    /// in: width, length, c2m (directions from camera to other reference frame, e.g. Manhattan)
    /// return: rectangle_area (in m^2)
    float getRectangleArea(Eigen::Matrix3f c2m);

    //// Compute extent, i.e. ratio of contour_area/rectangle_area
    /// return: extent (ratio between 0 and 1)
    float getExtent();

    //// Compute extent, i.e. ratio of contour_area/rectangle_area given specific directions
    /// in: c2m (directions from camera to other reference frame, e.g. Manhattan)
    /// return: extent (ratio between 0 and 1)
    float getExtent(Eigen::Matrix3f c2m);



    int type; // 0 = floor, 1 = horizontal, 2 = lateral wall, 3 = frontal wall, 5 = anything else
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud; // Plane cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2f; // Plane cloud in floor coordinates
    pcl::PointXYZ centroid; // Plane centroid
    pcl::PointXYZ centroid2f; // Plane centroid in floor coordinates
    Eigen::Vector4f coeffs; // Plane coefficients
    Eigen::Vector4f coeffs2f; // Plane coefficients in floor coordinates
    pcl::PointCloud<pcl::PointXYZ>::Ptr contour; // Contour (concave hull)
    pcl::PointCloud<pcl::PointXYZ>::Ptr contour2f; // Contour in floor coordinates
    Eigen::Matrix3f main_dir; // Principal directions of the plane
    float width, height, length; // Measurements of the bounding rectangle
    pcl::PointXYZ center; // Center of bounding rectangle
    pcl::PointCloud<pcl::PointXYZ>::Ptr vertices; // Vertices of the bounding rectangle

};


//// Function to perform sorting of planes by size
bool sortPlanesBySize(const Plane &lhs, const Plane &rhs);


#endif
