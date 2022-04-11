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

#include "mag_display.h"

#include <rviz_common/properties/status_property.hpp>
#include <rviz_common/logging.hpp>

namespace rviz_imu_plugin {

MagDisplay::MagDisplay()
    : twod_visual_(true), scene_node_(nullptr), messages_received_(0)
{
    createProperties();
}

void MagDisplay::onEnable()
{
    MessageFilterDisplay<sensor_msgs::msg::MagneticField>::onEnable();

    mag_visual_->show();

    scene_node_->setVisible(true);
}

void MagDisplay::onDisable()
{
    MessageFilterDisplay<sensor_msgs::msg::MagneticField>::onDisable();

    mag_visual_->hide();

    scene_node_->setVisible(false);
}

void MagDisplay::onInitialize()
{
    MFDClass::onInitialize();

    // Make an Ogre::SceneNode to contain all our visuals.
    scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();

    // create magnetic field heading vector visual
    mag_visual_ = new MagVisual(context_->getSceneManager(), scene_node_);

    scene_node_->setVisible(isEnabled());
}

MagDisplay::~MagDisplay() = default;

void MagDisplay::reset()
{
    MFDClass::reset();
    messages_received_ = 0;

    setStatus(rviz_common::properties::StatusProperty::Warn, "Topic",
              "No messages received");

    mag_visual_->hide();
}

void MagDisplay::update(float /* dt */, float /* ros_dt */)
{
    updateMag();
}

void MagDisplay::updateMag()
{
    if (isEnabled())
        mag_visual_->show();
    else
        mag_visual_->hide();

    mag_visual_->setScale(mag_scale_property_->getFloat());
    mag_visual_->setColor(mag_color_property_->getColor());
    mag_visual_->setAlpha(mag_alpha_property_->getFloat());
    mag_visual_->set2d(mag_2d_property_->getBool());
}

void MagDisplay::processMessage(
    const sensor_msgs::msg::MagneticField::ConstSharedPtr msg)
{
    if (!isEnabled()) return;

    ++messages_received_;

    std::stringstream ss;
    ss << messages_received_ << " messages received";
    setStatus(rviz_common::properties::StatusProperty::Ok, "Topic",
              ss.str().c_str());

    Ogre::Quaternion orientation;
    Ogre::Vector3 position;
    if (!context_->getFrameManager()->getTransform(
            msg->header.frame_id, msg->header.stamp, position, orientation))
    {
        RVIZ_COMMON_LOG_ERROR_STREAM("Error transforming from frame '"
                                     << msg->header.frame_id << "' to frame '"
                                     << fixed_frame_.toStdString() << "'");
        return;
    }

    mag_visual_->setMessage(msg);
    mag_visual_->setFramePosition(position);
    mag_visual_->setFrameOrientation(orientation);
    mag_visual_->show();
}

void MagDisplay::createProperties()
{
    // **** acceleration vector properties
    mag_2d_property_ = new rviz_common::properties::BoolProperty(
        "2D-visual", twod_visual_, "Use only 2D visualization", this,
        SLOT(updateMag()));

    mag_scale_property_ = new rviz_common::properties::FloatProperty(
        "Scale", true, "Vector size, in meters", this, SLOT(updateMag()));
    mag_color_property_ = new rviz_common::properties::ColorProperty(
        "Color", Qt::red, "Color to draw vector.", this, SLOT(updateMag()));
    mag_alpha_property_ = new rviz_common::properties::FloatProperty(
        "Alpha", 1.0, "0 is fully transparent, 1.0 is fully opaque.", this,
        SLOT(updateMag()));
}

}  // namespace rviz_imu_plugin

// Tell pluginlib about this class.  It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_imu_plugin::MagDisplay, rviz_common::Display)
