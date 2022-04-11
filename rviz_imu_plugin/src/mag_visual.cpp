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

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>

#include "mag_visual.h"
#include <rviz_rendering/objects/arrow.hpp>

namespace rviz_imu_plugin {

MagVisual::MagVisual(Ogre::SceneManager* scene_manager,
                     Ogre::SceneNode* parent_node)
    : heading_vector_(nullptr),
      arrow_length_(2.0),
      arrow_radius_(0.10),
      head_length_(0.20),
      head_radius_(0.10),
      scale_(0.05),
      alpha_(1.0),
      color_(1.0, 1.0, 0.0),
      is_2d_(true)
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

MagVisual::~MagVisual()
{
    hide();

    // Destroy the frame node since we don't need it anymore.
    scene_manager_->destroySceneNode(frame_node_);
}

void MagVisual::show()
{
    if (!heading_vector_)
    {
        heading_vector_ =
            new rviz_rendering::Arrow(scene_manager_, frame_node_);
        heading_vector_->setColor(color_.redF(), color_.greenF(),
                                  color_.blueF(), alpha_);
        heading_vector_->setDirection(direction_);
        heading_vector_->set(arrow_length_ * scale_, arrow_radius_ * scale_,
                             head_length_ * scale_, head_radius_ * scale_);
    }
}

void MagVisual::hide()
{
    if (heading_vector_)
    {
        delete heading_vector_;
        heading_vector_ = nullptr;
    }
}

void MagVisual::setMessage(
    const sensor_msgs::msg::MagneticField::ConstSharedPtr msg)
{
    if (is_2d_)
    {
        direction_ = Ogre::Vector3(msg->magnetic_field.x, msg->magnetic_field.y,
                                   0.0);  // msg->magnetic_field.z);
    } else
    {
        direction_ = Ogre::Vector3(msg->magnetic_field.x, msg->magnetic_field.y,
                                   msg->magnetic_field.z);
    }
    direction_.normalise();
    direction_ *= arrow_length_;

    if (heading_vector_)
    {
        heading_vector_->setDirection(direction_);
        heading_vector_->set(arrow_length_ * scale_, arrow_radius_ * scale_,
                             head_length_ * scale_, head_radius_ * scale_);
    }
}

void MagVisual::setScale(float scale)
{
    scale_ = scale;
    if (heading_vector_)
    {
        heading_vector_->setDirection(direction_);
        heading_vector_->set(arrow_length_ * scale_, arrow_radius_ * scale_,
                             head_length_ * scale_, head_radius_ * scale_);
    }
}

void MagVisual::setColor(const QColor& color)
{
    color_ = color;
    if (heading_vector_)
        heading_vector_->setColor(color_.redF(), color_.greenF(),
                                  color_.blueF(), alpha_);
}

void MagVisual::setAlpha(float alpha)
{
    alpha_ = alpha;
    if (heading_vector_)
        heading_vector_->setColor(color_.redF(), color_.greenF(),
                                  color_.blueF(), alpha_);
}

void MagVisual::setFramePosition(const Ogre::Vector3& position)
{
    frame_node_->setPosition(position);
}

void MagVisual::setFrameOrientation(const Ogre::Quaternion& orientation)
{
    frame_node_->setOrientation(orientation);
}

}  // namespace rviz_imu_plugin
