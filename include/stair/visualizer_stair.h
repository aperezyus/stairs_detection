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

#include "stair/stair_classes.h"
#include "RGBD/visualizer.h"
//#include "custom_functions.h"

struct ViewerStair : public Viewer
{
  ViewerStair()
  {


  }


  void drawStairAxis (Stair stair, std::string stair_type);
  void drawStairAxis (Stair stair, std::string stair_type, Eigen::Affine3d pose);
  void drawStairs (Stair stair, std::string stair_type);
  void drawStairs (Stair stair, std::string stair_type, Eigen::Affine3d pose);
  void drawRisers (Stair stair, std::string stair_type);
  void drawRisers (Stair stair, std::string stair_type, Eigen::Affine3d pose);
  void drawFullStairUntil (Stair stair, int level, Eigen::Affine3d s2p);
  void drawFullAscendingStairUntil (Stair stair, int level, Eigen::Affine3d s2p);
  void drawFullAscendingStairUntil (Stair stair, int level, Eigen::Affine3d s2p, Eigen::Affine3d pose);
  void drawFullDescendingStairUntil (Stair stair, int level, Eigen::Affine3d s2p);



};
