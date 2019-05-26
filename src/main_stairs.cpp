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

#include <math.h>
#include <iostream>
#include <signal.h>
#include <time.h>
#include <dirent.h> // To read directory

#include <boost/filesystem.hpp>
#include <boost/bind.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/topic.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/PCLHeader.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/common/time.h>

#include "stair/visualizer_stair.h"
#include "stair/global_scene_stair.h"
#include "stair/current_scene_stair.h"
#include "stair/stair_classes.h"


static int capture_mode = 0; // Capure mode can be 0 (reading clouds from ROS topic), 1 (reading from .pcd file), 2 (reading all *.pcd from directory)

void sayHelp(){
    std::cout << "-- Arguments to pass:" << std::endl;
    std::cout << "<no args>               - If no arguments ('$ rosrun stairs_detection stairs'), by default the algorithm proceed reading point clouds from ROS topic /camera/depth_registered/points" << std::endl;
    std::cout << "pcd <path to file>      - To run a PCD example (e.g. from Tang dataset), it should be '$ rosrun stairs_detection stairs pcd /path/to.pcd'" << std::endl;
    std::cout << "dir <path to directory> - To run all PCDs in a dataset, you can point at the folder, e.g. '$ rosrun stairs_detection stairs dir /path/to/pcds/" << std::endl;
}

// To run de program, create an object mainLoop
class mainLoop {
 public:
    mainLoop() : viewer(), gscene() {
        color_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
        color_cloud_show.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
        cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    }

    ~mainLoop() {}

    void cloudCallback(const sensor_msgs::PointCloud2 &cloud_msg) {
        pcl::fromROSMsg(cloud_msg,*color_cloud);
    }

    // This functions configures and executes the regular loop reading from a ROS topic (capture_mode = 0)
    void startMainLoop(int argc, char* argv[]) {
        // ROS subscribing
        ros::init(argc, argv, "kinect_navigator_node");
        ros::NodeHandle nh;

        ros::Subscriber cloud_sub = nh.subscribe("/camera/depth_registered/points", 1, &mainLoop::cloudCallback, this);

        int tries = 0;
        while (cloud_sub.getNumPublishers() == 0) {
            ROS_INFO("Waiting for subscibers");
            sleep(1);
            tries++;
            if (tries > 5){
                sayHelp();
                return;
            }
        }

        ros::Rate r(100);

        capture_.reset(new ros::AsyncSpinner(0));
        capture_->start();

        while (nh.ok() && !viewer.cloud_viewer_.wasStopped()) {
            if (color_cloud->points.size() > 0) {
                pcl::copyPointCloud(*color_cloud,*cloud);
                this->execute();
            }

            r.sleep();
        }
        capture_->stop();
    }

    // This functions configures and executes the loop reading from a .pcd file (capture_mode = 1)
    void startPCD(int argc, char* argv[]) {
        ros::init(argc, argv, "kinect_navigator_node");
        ros::NodeHandle nh;

        std::string cloud_file = argv[2];
        sensor_msgs::PointCloud2 input;
        pcl::io::loadPCDFile(cloud_file, input);
        pcl::fromROSMsg(input,*color_cloud);
        pcl::copyPointCloud(*color_cloud,*cloud);

        while (nh.ok()) {
            if (cloud->points.size() > 0)
                this->execute();
             if (viewer.cloud_viewer_.wasStopped())
                 break;
        }
    }

    // Helper function to read just *.pcd files in path
    bool has_suffix(const std::string& s, const std::string& suffix) {
        return (s.size() >= suffix.size()) && equal(suffix.rbegin(), suffix.rend(), s.rbegin());
    }

    // This functions configures and executes the loop reading all *.pcd files from a given directory (capture_mode = 2)
    void startDirectory(int argc, char* argv[]) {
        // ROS subscribing
        ros::init(argc, argv, "kinect_navigator_node");
        ros::NodeHandle nh;

        DIR *dir = opendir(argv[2]);
        if(dir){
            dirent *entry;
            while(((entry = readdir(dir))!=nullptr) && nh.ok()){
                if(has_suffix(entry->d_name, ".pcd")){
                    std::cout << "Loading pcd: " << entry->d_name << std::endl;
                    sensor_msgs::PointCloud2 input;
                    std::string full_path;
                    full_path.append(argv[2]);
                    full_path.append(entry->d_name);
                    pcl::io::loadPCDFile(full_path, input);
                    pcl::fromROSMsg(input,*color_cloud);
                    pcl::copyPointCloud(*color_cloud,*cloud);
                    if (cloud->points.size() > 0){
                        gscene.reset();
                        this->execute(); // To extract the floor
                        this->execute(); // To compute everything else
                    }

                }
            }
            closedir(dir);
        }
    }

     // Main loop to run the stairs detection and modelling algorithm for all modes.
    void execute() {
        // Prepare viewer for this iteration
        pcl::copyPointCloud(*color_cloud,*color_cloud_show);
        viewer.cloud_viewer_.removeAllPointClouds();
        viewer.cloud_viewer_.removeAllShapes();
        viewer.createAxis();

        // Process cloud from current view
        CurrentSceneStair scene;
        scene.applyVoxelFilter(0.04f, cloud); // Typically 0.04m voxels works fine for this method, however, bigger number (for more efficiency) or smaller (for more accuracy) can be used

        // The method first attempts to find the floor automatically. The floor position allows to orient the scene to reason about planar surfaces (including stairs)
        if (!gscene.initial_floor_) {
            gscene.findFloor(scene.fcloud);
            gscene.computeCamera2FloorMatrix(gscene.floor_normal_);
            viewer.drawAxis(gscene.f2c);
            viewer.drawColorCloud(color_cloud_show,1);
            viewer.cloud_viewer_.spinOnce();
        }
        else {
            // Compute the normals
//            scene.getNormalsNeighbors(8); // 8 Neighbours provides better accuracy, ideal for close distances (like the rosbag provided)
            scene.getNormalsNeighbors(16); // 16 works better for more irregular pointclouds, like those from far distances (e.g. Tang's dataset)
//            scene.getNormalsRadius(0.05f); // Alternatively, radius may be used instead of number of neighbors

            // Segment the scene in planes and clusters
            scene.regionGrowing();
            scene.extractClusters(scene.remaining_points);

            // Find and update the floor position
            gscene.findFloorFast(scene.vPlanes);
            if (gscene.new_floor_)
                gscene.computeCamera2FloorMatrix(gscene.floor_normal_);

            // Get centroids, contours and plane coefficients to floor reference
            scene.getCentroids();
            scene.getContours();
            scene.getPlaneCoeffs2Floor(gscene.c2f);
            scene.getCentroids2Floor(gscene.c2f);

            // Classify the planes in the scene
            scene.classifyPlanes();

            // Get Manhattan directions to rotate floor reference to also be aligned with vertical planes (OPTIONAL)
            gscene.getManhattanDirections(scene);

            // Some drawing functions for the PCL to see how the method is doing untill now
//                  viewer.drawNormals (scene.normals, scene.fcloud);
//                  viewer.drawPlaneTypesContour(scene.vPlanes);
//                  viewer.drawCloudsRandom(scene.vObstacles);
//                  viewer.drawAxis(gscene.f2c);

            // STAIR DETECTION AND MODELING
            if (scene.detectStairs()) { // First a quick check if horizontal planes may constitute staircases
                // Ascending staircase
                if (scene.getLevelsFromCandidates(scene.upstair,gscene.c2f)) { // Sort planes in levels
                    scene.upstair.modelStaircase(gscene.main_dir, gscene.has_manhattan_); // Perform the modeling
                    if (scene.upstair.validateStaircase()) { // Validate measurements
                        std::cout << "--- ASCENDING STAIRCASE ---\n" <<
                                     "- Steps: " << scene.upstair.vLevels.size()-1 << std::endl <<
                                     "- Measurements: " <<
                                     scene.upstair.step_width << "m of width, " <<
                                     scene.upstair.step_height << "m of height, " <<
                                     scene.upstair.step_length << "m of length. " << std::endl <<
                                     "- Pose (stair axis w.r.t. camera):\n" << scene.upstair.s2i.matrix() << std::endl << std::endl;

                        // Draw staircase
                        viewer.addStairsText(scene.upstair.i2s, gscene.f2c, scene.upstair.type);
                        viewer.drawFullAscendingStairUntil(scene.upstair,int(scene.upstair.vLevels.size()),scene.upstair.s2i);
                        viewer.drawStairAxis (scene.upstair, scene.upstair.type);
                    }

                }

                // Descending staircase
                if (scene.getLevelsFromCandidates(scene.downstair,gscene.c2f)) { // Sort planes in levels
                    scene.downstair.modelStaircase(gscene.main_dir, gscene.has_manhattan_); // Perform the modeling
                    if (scene.downstair.validateStaircase()) { // Validate measurements
                        std::cout << "--- DESCENDING STAIRCASE ---\n" <<
                                     "- Steps: " << scene.downstair.vLevels.size()-1 << std::endl <<
                                     "- Measurements: " <<
                                     scene.downstair.step_width << "m of width, " <<
                                     scene.downstair.step_height << "m of height, " <<
                                     scene.downstair.step_length << "m of length. " << std::endl <<
                                     "- Pose (stair axis w.r.t. camera):\n" << scene.downstair.s2i.matrix() << std::endl << std::endl;

                        // Draw staircase
                        viewer.addStairsText(scene.downstair.i2s, gscene.f2c, scene.downstair.type);
                        viewer.drawFullDescendingStairUntil(scene.downstair,int(scene.downstair.vLevels.size()),scene.downstair.s2i);
                        viewer.drawStairAxis (scene.downstair, scene.downstair.type);
                    }


                }

            }

            // Draw color cloud and update viewer
            viewer.drawColorCloud(color_cloud_show,1);
            if (capture_mode > 0)
                while(!viewer.cloud_viewer_.wasStopped())
                    viewer.cloud_viewer_.spinOnce();
            else
                viewer.cloud_viewer_.spinOnce();

        }
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_cloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_cloud_show; // just for visualization of each iteration, since color_cloud keeps being updated in the callback
    boost::shared_ptr<ros::AsyncSpinner> capture_;
    ViewerStair viewer; // Visualization object
    GlobalSceneStair gscene; // Global scene (i.e. functions and variables that should be kept through iterations)

};


void parseArguments(int argc, char ** argv, int &capture_mode){
    // Capture mode goes as follows:
    // 0 (default) - Read clouds from ROS topic '/camera/depth_registered/points/'
    // 1 - Reads pcd inserted as argument, e.g. 'pcd /path/to.pcd'
    // 2 - Reads all pcds from given directory, e.g. 'dir /path/to/pcds/
    capture_mode = 0;
    if (argc == 1) {
        capture_mode = 0; // From rosbag or live camera
    }
    else if (argc == 2) {
        if ((strcmp(argv[1], "h") == 0) or (strcmp(argv[1], "-h") == 0) or (strcmp(argv[1], "--h") == 0) or (strcmp(argv[1], "help") == 0) or (strcmp(argv[1], "-help") == 0) or (strcmp(argv[1], "--help") == 0)) {
            sayHelp();
        }
    }
    else if (argc == 3) {
        if (strcmp(argv[1], "pcd") == 0) {
            capture_mode = 1; // reads from PCD, which is the next argument

        }
        else if (strcmp(argv[1], "dir") == 0) {
            capture_mode = 2; // reads PCDs from directory
        }
    }
}



int main(int argc, char* argv[]) {
    mainLoop app;
    parseArguments(argc,argv,capture_mode);

    if (capture_mode == 0)
        app.startMainLoop(argc, argv);

    else if (capture_mode == 1){
        if (argc > 2)
            app.startPCD(argc, argv);
    }
    else if (capture_mode == 2){
        if (argc > 2)
            app.startDirectory(argc, argv);
    }

    return 0;
}
