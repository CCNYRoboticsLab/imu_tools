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

#include "imu_acc_visual.h"

namespace rviz
{

ImuAccVisual::ImuAccVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node):
  acc_vector_(NULL),
  arrow_length_(9.81),
  arrow_radius_(0.50),
  head_length_(1.00),
  head_radius_(1.00),
  scale_(0.05),
  alpha_(1.0),
  color_(1.0, 1.0, 0.0),
  derotated_(true)
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

ImuAccVisual::~ImuAccVisual()
{
  hide();

  // Destroy the frame node since we don't need it anymore.
  scene_manager_->destroySceneNode(frame_node_);
}

void ImuAccVisual::show()
{
  if (!acc_vector_)
  {
    acc_vector_ = new Arrow(scene_manager_, frame_node_);
    acc_vector_->setColor(color_.redF(), color_.greenF(), color_.blueF(), alpha_);
    acc_vector_->setDirection(direction_);
    acc_vector_->set(
      arrow_length_ * scale_, 
      arrow_radius_ * scale_, 
      head_length_  * scale_, 
      head_radius_  * scale_);
  }
}

void ImuAccVisual::hide()
{
  if (acc_vector_)
  {
    delete acc_vector_;
    acc_vector_ = NULL;
  }
}

void ImuAccVisual::setMessage(const sensor_msgs::Imu::ConstPtr& msg)
{
  direction_ = Ogre::Vector3(msg->linear_acceleration.x,
                             msg->linear_acceleration.y,
                             msg->linear_acceleration.z);


  // Rotate the acceleration vector by the IMU orientation. This makes
  // sense since the visualization of the IMU is also rotated by the 
  // orientation. In this way, both appear in the inertial frame.
  if (derotated_)
  {
    Ogre::Quaternion orientation(msg->orientation.w,
                                 msg->orientation.x,
                                 msg->orientation.y,
                                 msg->orientation.z);

    direction_ = orientation * direction_;
  }

  arrow_length_ = sqrt(
    msg->linear_acceleration.x * msg->linear_acceleration.x +
    msg->linear_acceleration.y * msg->linear_acceleration.y +
    msg->linear_acceleration.z * msg->linear_acceleration.z);

  if (acc_vector_)
  {
    acc_vector_->setDirection(direction_);
    acc_vector_->set(
      arrow_length_ * scale_, 
      arrow_radius_ * scale_, 
      head_length_  * scale_, 
      head_radius_  * scale_);
  }
}

void ImuAccVisual::setScale(float scale) 
{ 
  scale_ = scale; 
  if (acc_vector_)
  {
    acc_vector_->setDirection(direction_);
    acc_vector_->set(
      arrow_length_ * scale_, 
      arrow_radius_ * scale_, 
      head_length_  * scale_, 
      head_radius_  * scale_);
  }
}

void ImuAccVisual::setColor(const QColor& color)
{
  color_ = color;
  if (acc_vector_) 
    acc_vector_->setColor(color_.redF(), color_.greenF(), color_.blueF(), alpha_);
}

void ImuAccVisual::setAlpha(float alpha) 
{ 
  alpha_ = alpha; 
  if (acc_vector_) 
    acc_vector_->setColor(color_.redF(), color_.greenF(), color_.blueF(),alpha_);
}

void ImuAccVisual::setDerotated(bool derotated) 
{ 
  derotated_ = derotated; 
  if (acc_vector_) 
    acc_vector_->setColor(color_.redF(), color_.greenF(), color_.blueF(),alpha_);
}

void ImuAccVisual::setFramePosition(const Ogre::Vector3& position)
{
  frame_node_->setPosition(position);
}

void ImuAccVisual::setFrameOrientation(const Ogre::Quaternion& orientation)
{
  frame_node_->setOrientation(orientation);
}

} // end namespace rviz

