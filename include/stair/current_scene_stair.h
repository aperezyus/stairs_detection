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
#include "stair/stair_classes.h"


struct CurrentSceneStair : public CurrentScene
{

  CurrentSceneStair()
  {

  }

  ~CurrentSceneStair(){}


  bool checkForStairs();
  bool checkForStairsAbs();
  bool getLevelsFromCandidates(Stair & stair, Eigen::Affine3d c2f);

  Stair downstair;
  Stair upstair;

};


