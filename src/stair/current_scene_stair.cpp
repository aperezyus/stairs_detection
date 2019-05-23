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

#include "stair/current_scene_stair.h"

bool sortByHeight(const Plane &lstep, const Plane &rstep) {
  return fabs(lstep.centroid2f.y) < fabs(rstep.centroid2f.y); // <
}


bool CurrentSceneStair::detectStairs() {
    bool are_stairs = false;

    for (size_t Q = 0; Q<vPlanes.size(); Q++) {
        if (vPlanes[Q].type <= 1) {
            if (vPlanes[Q].centroid2f.y > k_height_min) {
                upstair.vPlanes.push_back(vPlanes[Q]);
                if (vPlanes[Q].centroid2f.y < k_height_max)
                    are_stairs = true;
            }
            else if (vPlanes[Q].centroid2f.y < -k_height_min) {
                downstair.vPlanes.push_back(vPlanes[Q]);
                if (vPlanes[Q].centroid2f.y > -k_height_max)
                    are_stairs = true;
            }
            else if ((vPlanes[Q].centroid2f.y > -k_height_min) and (vPlanes[Q].centroid2f.y < k_height_max)) {
                upstair.vPlanes.push_back(vPlanes[Q]);
                downstair.vPlanes.push_back(vPlanes[Q]);
            }
        }
    }
    return are_stairs;
}


bool CurrentSceneStair::getLevelsFromCandidates(Stair & stair, Eigen::Affine3d c2f) {

    if (stair.vPlanes.size() <= 1)
        return false;

    bool are_stairs = true;

    // Parameters:
    const float k_neighbour_search_radius = 0.50f; // To check connectivity, radius to search
    const int k_min_neighbour_points = 5; // How many points are considered enough to verify connectivity

    // Sort planes by height, closer to floor first
    sort(stair.vPlanes.begin(), stair.vPlanes.end(), sortByHeight);

    int max_level = 0;

    if (stair.vPlanes.size() > 1) {
        for (size_t Q=0; Q<(stair.vPlanes.size()); Q++) {
            if (fabs(stair.vPlanes[Q].centroid2f.y) < k_height_min) {
                // That's floor then
                if (stair.vLevels.size() == 0) {
                    // There were no floor yet, just add
                    Plane temp_plane(stair.vPlanes[Q].cloud, stair.vPlanes[Q].coeffs,stair.vPlanes[Q].centroid,stair.vPlanes[Q].centroid2f);
                    stair.vLevels.push_back(temp_plane);
                }
                else {
                    // There were floor, add pointcloud
                    *stair.vLevels[0].cloud += *stair.vPlanes[Q].cloud;
                }
            }
            else if (stair.vLevels.size() == 0) { // Step candidate is over the min threshold
                // There is no floor, this is straight Step 1
                // We create empty floor
                Plane temp_plane;
                stair.vLevels.push_back(temp_plane);

                if (fabs(stair.vPlanes[Q].centroid2f.y) < k_height_max) {
                    // First step!
                    Plane temp_plane(stair.vPlanes[Q].cloud, stair.vPlanes[Q].coeffs,stair.vPlanes[Q].centroid,stair.vPlanes[Q].centroid2f);
                    stair.vLevels.push_back(temp_plane);

                    max_level++;
                }
                else {
                    // First plane is higher than max threshold, thus there are no first step. False alarm.
                    return false;
                }
            }
            else if (stair.vLevels.size() == 1) { // Step candidate is over the min threshold, and there is floor
                if (fabs(stair.vPlanes[Q].centroid2f.y) < k_height_max) {
                    // Check neighbour connectivity
                    if (neighbour_search(stair.vLevels[0].cloud,stair.vPlanes[Q].centroid,k_neighbour_search_radius) > k_min_neighbour_points) {
                        // First step!
                        Plane temp_plane(stair.vPlanes[Q].cloud, stair.vPlanes[Q].coeffs,stair.vPlanes[Q].centroid,stair.vPlanes[Q].centroid2f);
                        stair.vLevels.push_back(temp_plane);

                        max_level++;
                    }
                }
                else {
                    // First plane is higher than max threshold, thus there are no first step. False alarm.
                    return false;
                }
            }
            else if (stair.vLevels.size() > 1) {
                // Generic case: there are levels established beforehand
                bool is_near = false;
                for (int P = max_level; P >= 0; P--) {
                    // Check connectivity with each previous level, starting from the top
                    if ((P == 0) and (stair.vLevels[size_t(P)].cloud->points.size() == 0))
                        break;

                    is_near = (neighbour_search(stair.vLevels[size_t(P)].cloud,stair.vPlanes[Q].centroid,k_neighbour_search_radius) > k_min_neighbour_points);

                    if (is_near)
                        break;
                }

                if (is_near) {
                    // Connected to previous step
                    if (fabs(fabs(stair.vPlanes[Q].centroid2f.y)-fabs(stair.vLevels[size_t(max_level)].centroid2f.y)) < k_height_threshold) {
                        // And within same height (thus, there are same level)
                        // Compute new normal of the level, by averaging depending on the size of the steps
                        Eigen::Vector3f new_normal = stair.vLevels[size_t(max_level)].coeffs.head(3)*stair.vLevels[size_t(max_level)].cloud->points.size() + stair.vPlanes[Q].coeffs.head(3)*stair.vPlanes[Q].cloud->points.size();
                        new_normal.normalize();
                        stair.vLevels[size_t(max_level)].coeffs.head(3) = new_normal;
                        *stair.vLevels[size_t(max_level)].cloud += *stair.vPlanes[Q].cloud;
                        stair.vLevels[size_t(max_level)].getCentroid();
                        stair.vLevels[size_t(max_level)].getCentroid2Floor(c2f);

                    }
                    else if (fabs(fabs(stair.vPlanes[Q].centroid2f.y)-fabs(stair.vLevels[size_t(max_level)].centroid2f.y)) > k_height_min &&
                             (fabs(fabs(stair.vPlanes[Q].centroid2f.y)-fabs(stair.vLevels[size_t(max_level)].centroid2f.y)) < k_height_max)) {
                        // They are at a valid range for new level!
                        Plane temp_plane(stair.vPlanes[Q].cloud, stair.vPlanes[Q].coeffs,stair.vPlanes[Q].centroid,stair.vPlanes[Q].centroid2f);
                        stair.vLevels.push_back(temp_plane);

                        max_level++;
                    }
                    // else.. ignore. They have no valid measurements.
                }
            }
        }
    }

    if (max_level < 1)
        are_stairs = false;

    return are_stairs;
}
