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
    // The height of the steps (as y coordinate of the transformed centroid to floor reference) is evaluated to see if they are within range
    // Besides, in this function the stair class is loaded with those planes that are step candidates (and floor)
    bool are_stairs = false;

    for (size_t Q = 0; Q<vPlanes.size(); Q++) {
        if (vPlanes[Q].type == 0) { // Floor is directly added
            upstair.vPlanes.push_back(vPlanes[Q]);
            downstair.vPlanes.push_back(vPlanes[Q]);
         }
        else if (vPlanes[Q].type == 1) { // Horizontal planes
            if (vPlanes[Q].centroid2f.y > k_height_min) { // High enough to be a step up
                upstair.vPlanes.push_back(vPlanes[Q]);
                if (vPlanes[Q].centroid2f.y < k_height_max) // function returns true if there is at least one first step candidate
                    are_stairs = true;
            }
            else if (vPlanes[Q].centroid2f.y < -k_height_min) { // Low enough to be a step down
                downstair.vPlanes.push_back(vPlanes[Q]);
                if (vPlanes[Q].centroid2f.y > -k_height_max) // function returns true if there is at least one first step candidate
                    are_stairs = true;
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
        else if (stair.vLevels.size() == 0) { // Step candidate is over the min threshold, and there is no floor, this is straight Step 1
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
//                if (neighbour_search(stair.vLevels[0].cloud,stair.vPlanes[Q].centroid,k_neighbour_search_radius) > k_min_neighbour_points) {
                    // First step!
                    Plane temp_plane(stair.vPlanes[Q].cloud, stair.vPlanes[Q].coeffs,stair.vPlanes[Q].centroid,stair.vPlanes[Q].centroid2f);
                    stair.vLevels.push_back(temp_plane);

                    max_level++;
//                }
            }
            else {
                // First plane is higher than max threshold, thus there are no first step. False alarm.
                return false;
            }
        }
        else if (stair.vLevels.size() > 1) {
            // Generic case: there are levels established beforehand. If it is not near to any before, the function loops to find another one
            bool is_near_max_level = false;
            bool is_near_prev_level = false;

            // Compute connectivity with current max level and previous levels
            is_near_max_level = (neighbour_search(stair.vLevels[size_t(max_level)].cloud,stair.vPlanes[Q].centroid,k_neighbour_search_radius) > k_min_neighbour_points);
            for (int P = max_level -1; P >= 0; P--) {
                // Check connectivity with each previous level, starting from the top minus one
                if ((P == 0) and (stair.vLevels[size_t(P)].cloud->points.size() == 0))
                    break;

                is_near_prev_level = (neighbour_search(stair.vLevels[size_t(P)].cloud,stair.vPlanes[Q].centroid,k_neighbour_search_radius) > k_min_neighbour_points);

                if (is_near_prev_level)
                    break;
            }

            // If it is not connected to anything, loop with other step candidate
            if (!is_near_max_level && !is_near_prev_level)
                continue;

            // Now check heights: Similar height?
            if (fabs(fabs(stair.vPlanes[Q].centroid2f.y)-fabs(stair.vLevels[size_t(max_level)].centroid2f.y)) < k_height_threshold) {
                if (is_near_max_level) {
                    // They are close, proceed to fuse.
                    // Compute new normal of the level, by averaging depending on the size of the steps
                    Eigen::Vector3f new_normal = stair.vLevels[size_t(max_level)].coeffs.head(3)*stair.vLevels[size_t(max_level)].cloud->points.size() + stair.vPlanes[Q].coeffs.head(3)*stair.vPlanes[Q].cloud->points.size();
                    new_normal.normalize();
                    stair.vLevels[size_t(max_level)].coeffs.head(3) = new_normal;
                    *stair.vLevels[size_t(max_level)].cloud += *stair.vPlanes[Q].cloud;
                    stair.vLevels[size_t(max_level)].getCentroid();
                    stair.vLevels[size_t(max_level)].getCentroid2Floor(c2f);
                }
                else { // not close to current max level
                    if (max_level > 1) {  // It is not with current level but it is with previous (otherwise we would have exit the loop)
                        // They belong to the same step although occluded, proceed to fuse.
                        // Compute new normal of the level, by averaging depending on the size of the steps
                        Eigen::Vector3f new_normal = stair.vLevels[size_t(max_level)].coeffs.head(3)*stair.vLevels[size_t(max_level)].cloud->points.size() + stair.vPlanes[Q].coeffs.head(3)*stair.vPlanes[Q].cloud->points.size();
                        new_normal.normalize();
                        stair.vLevels[size_t(max_level)].coeffs.head(3) = new_normal;
                        *stair.vLevels[size_t(max_level)].cloud += *stair.vPlanes[Q].cloud;
                        stair.vLevels[size_t(max_level)].getCentroid();
                        stair.vLevels[size_t(max_level)].getCentroid2Floor(c2f);
                    }
                    else { // Max level == 1
                        // Since the planes are not close enough in the same level, the only plausible connection is with the floor
                        // Thus, it could be a case of occlusion or two isolated staircases or another plane that could deceive the method
                        // Here we are conservative, considering we only obtain one staircase at a time. So just at this level, if they are not close
                        // in the radius predefined by k_neighbour_search_radius, we chose between one of the candidates
                        // (the already established as level 1 or the newest step candidate vPlanes[Q])
                        if (fabs(stair.vPlanes[Q].centroid2f.y) < k_height_max) { // Make sure it is not too large for first step
                            // Now, to verify if it is a better "path", first analyze which one is connected to further steps:
//                            if (Q<(stair.vPlanes.size())-1) {   // Unless it is the last one, of course
                            bool is_near_level = false; // "level" means the already established level
                            bool is_near_plane = false; // "plane" means the new step candidate
                            for (size_t P = Q+1; P < stair.vPlanes.size(); P++) {
                                if (!is_near_level)
                                    is_near_level = (neighbour_search(stair.vPlanes[P].cloud,stair.vLevels[size_t(max_level)].centroid,k_neighbour_search_radius) > k_min_neighbour_points);
                                if (!is_near_plane)
                                    is_near_plane = (neighbour_search(stair.vPlanes[P].cloud,stair.vPlanes[Q].centroid,k_neighbour_search_radius) > k_min_neighbour_points);

                                if (is_near_plane && is_near_level)
                                    break;
                            }

                            if (is_near_plane && !is_near_level) { // If the new one is connected to further stages but the old one is not, substitute to the new one
                                stair.vLevels[size_t(max_level)] = stair.vPlanes[Q];
                            }
                            else if (is_near_level && is_near_plane){
                                // Both are further steps! There are several conditions we may use to consider now to which one is best: size, extent, distance...
                                // In practice, it depends on the scene. Let's use size, for example:
                                // if (stair.vLevels[size_t(max_level)].centroid.getVector3fMap().norm() > stair.vPlanes[Q].centroid.getVector3fMap().norm()){ // Distance
                                // if (stair.vLevels[size_t(max_level)].getRectangleArea() < stair.vPlanes[Q].getRectangleArea()) { // Rectangle area
                                if (stair.vLevels[size_t(max_level)].cloud->points.size() < stair.vPlanes[Q].cloud->points.size()) { // Size of the cloud
                                    // They are not close, and the new one is bigger. Discard the old one.
                                    stair.vLevels[size_t(max_level)] = stair.vPlanes[Q];
                                }
                            }
                            else if (!is_near_level && !is_near_plane) {
                                // Since there are no more planes connected, you can keep the largest
                                if (stair.vLevels[size_t(max_level)].cloud->points.size() < stair.vPlanes[Q].cloud->points.size()) // Size of the cloud
                                    // They are not close, and the new one is bigger. Discard the old one.
                                    stair.vLevels[size_t(max_level)] = stair.vPlanes[Q];
                            }
                                // Else leave level be (has more future), discard new plane (i.e. do nothing and loop)
//                            }
                        }// else discard, too close to established level to actually be a new level
                    }
                }
            }
            else if (max_level == 1 && (fabs(stair.vPlanes[Q].centroid2f.y) < k_height_max)) { // Notice that it is not necessary to be within height range to enter here
                     // Now, to verify if it is a better "path", first analyze which one is connected to further steps:
                     bool is_near_level = false; // "level" means the already established level
                     bool is_near_plane = false; // "plane" means the new step candidate
                     for (size_t P = Q+1; P < stair.vPlanes.size(); P++) {
                         if (!is_near_level)
                             is_near_level = (neighbour_search(stair.vPlanes[P].cloud,stair.vLevels[size_t(max_level)].centroid,k_neighbour_search_radius) > k_min_neighbour_points);
                         if (!is_near_plane)
                             is_near_plane = (neighbour_search(stair.vPlanes[P].cloud,stair.vPlanes[Q].centroid,k_neighbour_search_radius) > k_min_neighbour_points);

                         if (is_near_plane && is_near_level)
                             break;
                     }

                     if (is_near_plane && !is_near_level) { // If the new one is connected to further stages but the old one is not, substitute to the new one
                         stair.vLevels[size_t(max_level)] = stair.vPlanes[Q];
                     }
                     else if (is_near_level && is_near_plane){
                         // Both are further steps! There are several conditions we may use to consider now to which one is best: size, extent, distance...
                         // In practice, it depends on the scene. Let's use size, for example:
                         // if (stair.vLevels[size_t(max_level)].centroid.getVector3fMap().norm() > stair.vPlanes[Q].centroid.getVector3fMap().norm()){ // Distance
                         // if (stair.vLevels[size_t(max_level)].getRectangleArea() < stair.vPlanes[Q].getRectangleArea()) { // Rectangle area
                         if (stair.vLevels[size_t(max_level)].cloud->points.size() < stair.vPlanes[Q].cloud->points.size()) { // Size of the cloud
                             // They are not close, and the new one is bigger. Discard the old one.
                             stair.vLevels[size_t(max_level)] = stair.vPlanes[Q];
                         }
                     }
                     else if (!is_near_level && !is_near_plane) {
                         // Since there are no more planes connected, you can keep the largest
                         if (stair.vLevels[size_t(max_level)].cloud->points.size() < stair.vPlanes[Q].cloud->points.size()) // Size of the cloud
                             // They are not close, and the new one is bigger. Discard the old one.
                             stair.vLevels[size_t(max_level)] = stair.vPlanes[Q];
                     }
            }
            else if (fabs(fabs(stair.vPlanes[Q].centroid2f.y)-fabs(stair.vLevels[size_t(max_level)].centroid2f.y)) > k_height_min &&
                     (fabs(fabs(stair.vPlanes[Q].centroid2f.y)-fabs(stair.vLevels[size_t(max_level)].centroid2f.y)) < k_height_max)) {
                // They are at a valid range for new level!
                Plane temp_plane(stair.vPlanes[Q].cloud, stair.vPlanes[Q].coeffs,stair.vPlanes[Q].centroid,stair.vPlanes[Q].centroid2f);
                stair.vLevels.push_back(temp_plane);

                max_level++;
            }
        }
    }

    if (max_level < 1)
        are_stairs = false;

    return are_stairs;
}
