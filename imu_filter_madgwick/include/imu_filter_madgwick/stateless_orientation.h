/*
 *  Copyright (C) 2010, CCNY Robotics Lab
 *  Ivan Dryanovski <ivan.dryanovski@gmail.com>
 *
 *  http://robotics.ccny.cuny.edu
 *
 *  Based on implementation of Madgwick's IMU and AHRS algorithms.
 *  http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
 *
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef IMU_FILTER_MADWICK_STATELESS_ORIENTATION_H
#define IMU_FILTER_MADWICK_STATELESS_ORIENTATION_H

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <imu_filter_madgwick/world_frame.h>

class StatelessOrientation
{
  public:
    static bool computeOrientation(WorldFrame::WorldFrame frame,
                                   geometry_msgs::Vector3 acceleration,
                                   geometry_msgs::Vector3 magneticField,
                                   geometry_msgs::Quaternion& orientation);

    static bool computeOrientation(WorldFrame::WorldFrame frame,
                                   geometry_msgs::Vector3 acceleration,
                                   geometry_msgs::Quaternion& orientation);
};

#endif  // IMU_FILTER_MADWICK_STATELESS_ORIENTATION_H
