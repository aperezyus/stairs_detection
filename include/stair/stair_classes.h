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

#ifndef STAIR_CLASSES_H
#define STAIR_CLASSES_H

#include <boost/filesystem.hpp>
#include <pcl/PCLHeader.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common_headers.h>

#include "RGBD/plane.h"

// Parameters for height of the stairs, given by the regulations in our country (Spain):
const float k_height_threshold = 0.04f;   // Around voxel grid value
const float k_length_threshold = 0.06f;   // Some stairs in the dataset have lengths around 23cm, thus I had to increase the threshold to be safe
const float k_height_min = 0.13f - k_height_threshold; // Min height is 13 cm
const float k_height_max = 0.185f + k_height_threshold; // Max height is 18.5 cm
const float k_length_min = 0.28f - k_length_threshold; // Min length is 28 cm (no max length)
const float k_sum_min = 0.54f - 2*k_height_threshold - k_length_threshold; // Sum of two risers and the length of one step must sum more than 54 cm
const float k_sum_max = 0.70f + 2*k_height_threshold + k_length_threshold; // Sum of two risers and the length of one step must sum less than 70 cm

int neighbour_search(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointXYZ point, float radius, pcl::PointCloud<pcl::PointXYZ>::Ptr &neighbouring_cloud);
int neighbour_search(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointXYZ point, float radius);

class Stair {
  public:

    Stair()	{
        step_height = 0;
        step_width = 0;
        step_length = 0;
    }
    ~Stair(){}
	
    //// Chooses the best step (as a type Plane) from all the planes found as the one that produces greater extent from all candidates
    /// in: vLevels (from the detection stage)
    /// returns: best_step (type Plane)
    Plane getBestStep ();

    //// Given a step, compute the main_dir of the plane considering our axes convention (i.e. Z frontwards, Y upwards)
    /// in: plane (a type Plane, used to find best_step))
    /// out: plane.main_dir (oriented according to convention)
    void getStairDirFromPlane(Plane &plane);

    //// Gets the first estimation of the staircase as a whole: length, width, height, pose and initial point
    ///  Two methods are proposed, one standalone based on PCA, other using Manhattan dirs.
    ///  This fuction computes both and chooses the best one.
    /// in: manhattan_dirs (computed from GlobalScene), standalone_dirs (e.g. main_dirs from best_step), has_manhattan (to avoid compute manhattan)
    /// out: (in class): i2s and s2i (pose from camera at pose i to stair and inverse), stair_dir (axes of the stair reference frame), width, length, height (of the staircase), center (of the cuboid volume)
    void getInitialStairVolume(Eigen::Matrix3f manhattan_dirs, Eigen::Matrix3f standalone_dirs, bool has_manhattan);

    //// Computes the 5 vertices of the step given following shape, with axes from stair_dir:
    /// 0--------------1
    /// |       z      |
    /// |       |      |
    /// |    x--4      |
    /// |              |
    /// 3--------------2
    /// in: stair_dir, vLevels
    /// out: vLevels.vertices
	void getInitialStepVertices();

    //// Computes the 5 vertices of the step given following shape, with axes from given dir:
    /// 0--------------1
    /// |       z      |
    /// |       |      |
    /// |    x--4      |
    /// |              |
    /// 3--------------2
    /// in: dir (matrix, can be stair_dir), vLevels
    /// out: vLevels.vertices
	void getInitialStepVertices(Eigen::Matrix3f & dir);

    //// Computes final vertices of the whole staircase, considering occlusions of the view, and averaging heights and lengths.
    /// in: whole Stair class
    /// out: vOrientedLevels, which are the vertices already oriented to be displayed in viewer.
	void getExactStepVertices();

    //// Function that executes all previously defined functions of the class at once, providing a final Stair object with all necessary elements
    void modelStaircase(Eigen::Matrix3f main_dir, bool has_manhattan);

    //// Function to validate staircase detection according to regulations measurements with threshold given above.
    /// returns: true if valid measurements
    bool validateStaircase();
	
    std::vector<Plane> vPlanes; // Step candidates given by the detection process
    std::vector<Plane> vLevels; // Step candidates ordered in "levels" regarding their distance to the floor in steps
    std::vector<Plane> vOrientedLevels; // Simple vector that includes final vertex values of the staircase (e.g. to plot)
    std::vector<Plane> vRisers; // Includes final vertices but in this case for the risers (not used for computation)
    Eigen::Matrix3f stair_dir; // Main directions of the staircase given our convention
    Eigen::Affine3d i2s; // Transformation matrix to transform points from camera view (at frame "i") to stair coordinates
    Eigen::Affine3d s2i; // Inverse of i2s
    pcl::PointXYZ initial_point; // Point at the center of the edge of the first step, origin of stair reference frame
	
    float step_width; // Width of the step (from left to right when in front of it)
    float step_length; // Length of the step (front to back)
    float step_height; // Height of the step

    std::string type; // Can be "up" or "down" depending if it is ascending or descending staircase


};

#endif
