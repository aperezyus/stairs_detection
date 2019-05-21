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

//#define _USE_MATH_DEFINES

#include <math.h>
#include <iostream>
#include <signal.h>
#include <time.h>

#include <boost/filesystem.hpp>
#include <boost/bind.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/topic.h>

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <gazebo_msgs/LinkStates.h>
#include <nav_msgs/Odometry.h>
#include <ros/callback_queue.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
#include <eigen_conversions/eigen_msg.h>

#include <pcl/PCLHeader.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/time.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include "stair/visualizer_stair.h"
#include "stair/global_scene_stair.h"
#include "stair/current_scene_stair.h"
#include "stair/stair_classes.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/eigen.hpp>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>


struct mainLoop
{
  mainLoop() : gscene(), viewer()
  {
//    viewer.createAxis();
//    gscene.global_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
//    gscene.cumulated_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);

  }

  ~mainLoop() {}

  void cloudCallback(const sensor_msgs::PointCloud2 &cloud_msg)
  {
      boost::mutex::scoped_try_lock lock(data_ready_mutex_);
      if (!lock)
        return;

      color_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
      cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::fromROSMsg(cloud_msg,*color_cloud);
      pcl::copyPointCloud(*color_cloud,*cloud);

      data_ready_cond_.notify_one();

  }

  void startMainLoop(int argc, char* argv[])
  {

    // ROS subscribing
    ros::init(argc, argv, "kinect_navigator_node");
    ros::NodeHandle nh;

    ros::Subscriber cloud_sub = nh.subscribe("/camera/depth_registered/points", 1, &mainLoop::cloudCallback, this);

    int tries = 0;
    while (cloud_sub.getNumPublishers() == 0)
    {
      ROS_INFO("Waiting for subscibers");
      sleep(1);
      tries++;
      if (tries > 5)
        return;
    }

    capture_.reset(new ros::AsyncSpinner(0));

    {
      boost::unique_lock<boost::mutex> lock(data_ready_mutex_);
      capture_->start();

      while (nh.ok())
      {
        bool has_data = data_ready_cond_.timed_wait(lock, boost::posix_time::millisec(500));
        if (has_data && cloud->points.size() > 0)
          this->execute();
      }
    }
  }

  void startPCD(int argc, char* argv[])
  {
//     ROS subscribing
    ros::init(argc, argv, "kinect_navigator_node");
    ros::NodeHandle nh;

    color_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);

    std::string cloud_file = argv[2];
    sensor_msgs::PointCloud2 input;
    pcl::io::loadPCDFile(cloud_file, input);
    pcl::fromROSMsg(input,*color_cloud);
    pcl::copyPointCloud(*color_cloud,*cloud);




//    ros::Subscriber cloud_sub = nh.subscribe("/camera/depth_registered/points", 1, &mainLoop::cloudCallback, this);

//    int tries = 0;
//    while (cloud_sub.getNumPublishers() == 0)
//    {
//      ROS_INFO("Waiting for subscibers");
//      sleep(1);
//      tries++;
//      if (tries > 5)
//        return;
//    }

//    capture_.reset(new ros::AsyncSpinner(0));

//    {
//      boost::unique_lock<boost::mutex> lock(data_ready_mutex_);
//      capture_->start();

      while (nh.ok())
      {
//        bool has_data = data_ready_cond_.timed_wait(lock, boost::posix_time::millisec(500));
        if (cloud->points.size() > 0)
          this->execute();
      }
//    }
  }

  void execute()
  {

    viewer.cloud_viewer_.removeAllPointClouds();
    viewer.cloud_viewer_.removeAllShapes();

    viewer.createAxis();


    CurrentSceneStair scene;
    scene.applyVoxelFilter(0.04f, cloud);
    if (!gscene.initial_floor)
    {
      gscene.findFloor(scene.fcloud);
      gscene.computeCamera2FloorMatrix();
      viewer.drawAxis(gscene.f2c);

    }
    else
    {
      scene.getNormalsNeighbors(16);
//      scene.getNormalsRadius(0.05f);
      scene.regionGrowing();
      scene.extractClusters(scene.remaining_points);
      gscene.findFloorFast(scene.vPlanes);
      if (gscene.new_floor)
        gscene.computeCamera2FloorMatrix();
      scene.getCentroids();
      scene.getContours();
      scene.getPlaneCoeffs2Floor(gscene.c2f);
      scene.getCentroids2Floor(gscene.c2f);
      scene.classifyPlanes();
      gscene.getManhattanDirections(scene);
//      if (gscene.has_manhattan)
//      {
//        scene.transformPlanesAndObstacles(gscene.c2f, false);
//      }

//      std::cout << gscene.c2f.matrix() << std::endl;


//      viewer.drawNormals (scene.normals, scene.fcloud);
      viewer.drawPlaneTypesContour(scene.vPlanes);
//      viewer.drawCloudsRandom(scene.vObstacles);
      viewer.drawAxis(gscene.f2c);

      if (scene.checkForStairs())
      {
        if (scene.getLevelsFromCandidates(scene.upstair,gscene.c2f))
        {
          scene.upstair.type = "up";
          Plane best_step = scene.upstair.getBestStep(gscene.c2f);
          scene.upstair.getInitialStairVolume(gscene.eigDx, best_step.eigDx, scene.has_manhattan);
          scene.upstair.getInitialStepVertices();
          gscene.f2s = scene.upstair.i2s * gscene.f2c;
          double x,y,z,roll,pitch,yaw;
          pcl::getTranslationAndEulerAngles (gscene.f2s, x, y, z, roll, pitch, yaw);
          std::stringstream ss;
          ss << "Ascending stairs at " << -z << "m and " << pitch*180/M_PI << "º";
          viewer.cloud_viewer_.addText (ss.str(), 50, 50, 20, 1.0f, 1.0f, 1.0f, "uptext");
          scene.upstair.getExactStepVertices();
          viewer.drawFullAscendingStairUntil(scene.upstair,scene.upstair.vLevels.size(),scene.upstair.s2i);
          viewer.drawStairAxis (scene.upstair, scene.upstair.type);
//          viewer.drawStairs(scene.upstair,scene.upstair.type);
        }

        if (scene.getLevelsFromCandidates(scene.downstair,gscene.c2f))
        {
          scene.downstair.type = "down";
          Plane best_step = scene.downstair.getBestStep(gscene.c2f);
          scene.downstair.getInitialStairVolume(gscene.eigDx, best_step.eigDx, scene.has_manhattan);
          scene.downstair.getInitialStepVertices();
          gscene.f2s = scene.downstair.i2s * gscene.f2c;
          double x,y,z,roll,pitch,yaw;
          pcl::getTranslationAndEulerAngles (gscene.f2s, x, y, z, roll, pitch, yaw);
          std::stringstream ss;
          ss << "Descending stairs at " << -z << "m and " << pitch*180/M_PI << "º";
          viewer.cloud_viewer_.addText (ss.str(), 50, 100, 20, 1.0f, 1.0f, 1.0f, "downtext");
          scene.downstair.getExactStepVertices();
          viewer.drawFullDescendingStairUntil(scene.downstair,scene.downstair.vLevels.size(),scene.downstair.s2i);
          viewer.drawStairAxis (scene.upstair, scene.upstair.type);

        }

      }
      else
      {
          viewer.cloud_viewer_.addText ("No ascending stairs", 50, 50, 20, 1.0f, 1.0f, 1.0f, "uptext");
          viewer.cloud_viewer_.addText ("No descending stairs", 50, 100, 20, 1.0f, 1.0f, 1.0f, "downtext");
      }

    }


    viewer.drawColorCloud(color_cloud,1);
//                viewer.cloud_viewer_.spin();
    viewer.cloud_viewer_.spinOnce();

  }

  // ROS/RGB-D
  boost::mutex data_ready_mutex_;
  boost::condition_variable data_ready_cond_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_cloud;
  boost::shared_ptr<ros::AsyncSpinner> capture_;

  // RGB-D
  ViewerStair viewer;
  GlobalSceneStair gscene;

};

void parseArguments(int argc, char ** argv, int &capture_mode)
{
//std::cout << argc << std::endl;

    capture_mode = 0;
    if (argc == 1)
    {
        capture_mode = 1; // From rosbag or live camera
    }
    else if (argc == 2)
    {
        if ((strcmp(argv[1], "h") == 0) or (strcmp(argv[1], "-h") == 0) or (strcmp(argv[1], "--h") == 0) or (strcmp(argv[1], "help") == 0) or (strcmp(argv[1], "-help") == 0) or (strcmp(argv[1], "--help") == 0))
        {
            std::cout << "USAGE" << std::endl;
            std::cout << "./navegador_interiores_alejandro old : uses old openni driver with /camera/rgb/points as topic for the PointClouds" << std::endl;
            std::cout << "./navegador_interiores_alejandro new : uses new openni2 driver with /camera/depth_registered/points as topic for the PointClouds" << std::endl;
            std::cout << "./navegador_interiores_alejandro : uses new openni2 driver with /camera/depth_registered/points as topic for the PointClouds" << std::endl;
            std::cout << "./navegador_interiores_alejandro pcd filename.pcd: loads PointCloud from filename.pcd" << std::endl;
        }
    }
    else if (argc == 3)
    {
        if (strcmp(argv[1], "pcd") == 0)
        {
            capture_mode = 0; // reads from PCD, which is the next argument

        }
    }
}

int main(int argc, char* argv[])
{
  mainLoop app;
  int capture_mode;
  parseArguments(argc,argv,capture_mode);

  if (capture_mode == 1)
  {
      try
      {
        app.startMainLoop(argc, argv);
      }
      catch (const std::bad_alloc& /*e*/)
      {
        cout << "Bad alloc" << endl;
      }
      catch (const std::exception& /*e*/)
      {
        cout << "Exception" << endl;
      }
  }
  else
  {
      if (argc > 2)
      {
          try
          {
            app.startPCD(argc, argv);
          }
          catch (const std::bad_alloc& /*e*/)
          {
            cout << "Bad alloc" << endl;
          }
          catch (const std::exception& /*e*/)
          {
            cout << "Exception" << endl;
          }
      }
  }

  return 0;
}
