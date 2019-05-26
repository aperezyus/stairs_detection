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

#ifndef CURRENT_SCENE_STAIR_H
#define CURRENT_SCENE_STAIR_H


#include "RGBD/current_scene.h"
#include "stair/stair_classes.h"



class CurrentSceneStair : public CurrentScene {
  public:
    CurrentSceneStair(){
        upstair.type = "up";
        downstair.type = "down";
    }

    ~CurrentSceneStair(){}

    //// Goes through all horizontal planes and return positive if there are at least one valid first step
    /// Step candidates are passed to Stair object (upstair and downstair at the same time)
    /// in: vPlanes
    /// out: stair.vPlanes
    /// return true if there are first step candidate
    bool detectStairs();


    //// Sorts step candidate regions in levels, being those a vector of Planes that are sorted using distance w.r.t. floor
    /// in: Stair object to sort elements with, c2f (transformation matrix from camera to floor coordinates)
    /// out: stair.vLevels
    /// return true if valid staircase found
    bool getLevelsFromCandidates(Stair & stair, Eigen::Affine3d c2f);

    // Stair objects for down and up staircases. One for each implemented.
    Stair downstair;
    Stair upstair;

};


#endif
