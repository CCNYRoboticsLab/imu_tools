/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * Copyright (c) 2012, Ivan Dryanovski <ivan.dryanovski@gmail.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "imu_orientation_visual.h"

#include <ros/ros.h>
#include <cmath>

namespace rviz
{

ImuOrientationVisual::ImuOrientationVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node):
  orientation_box_(NULL),
  scale_x_(0.07),
  scale_y_(0.10),
  scale_z_(0.03),
  alpha_(1.0),
  quat_valid_(true),
  color_(0.5, 0.5, 0.5)
{
  scene_manager_ = scene_manager;

  // Ogre::SceneNode s form a tree, with each node storing the
  // transform (position and orientation) of itself relative to its
  // parent.  Ogre does the math of combining those transforms when it
  // is time to render.
  //
  // Here we create a node to store the pose of the Imu's header frame
  // relative to the RViz fixed frame.
  frame_node_ = parent_node->createChildSceneNode();
}

ImuOrientationVisual::~ImuOrientationVisual()
{
  hide();

  // Destroy the frame node since we don't need it anymore.
  scene_manager_->destroySceneNode(frame_node_);
}

void ImuOrientationVisual::show()
{
  if (!orientation_box_)
  {
    orientation_box_ = new Shape(Shape::Cube, scene_manager_, frame_node_);
    orientation_box_->setColor(color_.redF(), color_.greenF(), color_.blueF(), alpha_);
    orientation_box_->setScale(Ogre::Vector3(scale_x_, scale_y_, scale_z_));
    orientation_box_->setOrientation(orientation_);
  }
}

void ImuOrientationVisual::hide()
{
  if (orientation_box_)
  {
    delete orientation_box_;
    orientation_box_ = NULL;
  }
}

void ImuOrientationVisual::setMessage(const sensor_msgs::Imu::ConstPtr& msg)
{
  if (checkQuaternionValidity(msg)) {
    if (!quat_valid_) {
      ROS_INFO("rviz_imu_plugin got valid quaternion, "
               "displaying true orientation");
      quat_valid_ = true;
    }
    orientation_ = Ogre::Quaternion(msg->orientation.w,
                                    msg->orientation.x,
                                    msg->orientation.y,
                                    msg->orientation.z);
  } else {
    if (quat_valid_) {
      ROS_WARN("rviz_imu_plugin got invalid quaternion (%lf, %lf, %lf, %lf), "
               "will display neutral orientation instead", msg->orientation.w,
               msg->orientation.x,msg->orientation.y,msg->orientation.z);
      quat_valid_ = false;
    }
    // if quaternion is invalid, give a unit quat to Ogre
    orientation_ = Ogre::Quaternion();
  }

  if (orientation_box_)
    orientation_box_->setOrientation(orientation_);
}

void ImuOrientationVisual::setScaleX(float x) 
{ 
  scale_x_ = x; 
  if (orientation_box_) 
   orientation_box_->setScale(Ogre::Vector3(scale_x_, scale_y_, scale_z_));
}

void ImuOrientationVisual::setScaleY(float y) 
{ 
  scale_y_ = y; 
  if (orientation_box_) 
    orientation_box_->setScale(Ogre::Vector3(scale_x_, scale_y_, scale_z_));
}

void ImuOrientationVisual::setScaleZ(float z) 
{ 
  scale_z_ = z; 
  if (orientation_box_) 
    orientation_box_->setScale(Ogre::Vector3(scale_x_, scale_y_, scale_z_));

}

void ImuOrientationVisual::setColor(const QColor& color)
{
  color_ = color;
  if (orientation_box_) 
    orientation_box_->setColor(color_.redF(), color_.greenF(), color_.blueF(), alpha_);
}

void ImuOrientationVisual::setAlpha(float alpha) 
{ 
  alpha_ = alpha; 
  if (orientation_box_) 
    orientation_box_->setColor(color_.redF(), color_.greenF(), color_.blueF(), alpha_);
}

void ImuOrientationVisual::setFramePosition(const Ogre::Vector3& position)
{
  frame_node_->setPosition(position);
}

void ImuOrientationVisual::setFrameOrientation(const Ogre::Quaternion& orientation)
{
  frame_node_->setOrientation(orientation);
}

inline bool ImuOrientationVisual::checkQuaternionValidity(
    const sensor_msgs::Imu::ConstPtr& msg) {

  double x = msg->orientation.x,
         y = msg->orientation.y,
         z = msg->orientation.z,
         w = msg->orientation.w;
  // OGRE can handle unnormalized quaternions, but quat's length extremely small;
  // this may indicate that invalid (0, 0, 0, 0) quat is passed, this will lead ogre
  // to crash unexpectly
  if ( std::sqrt( x*x + y*y + z*z + w*w ) < 0.0001 ) {
    return false;
  }
  return true;
}


} // end namespace rviz

