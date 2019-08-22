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

#include "imu_display.h"

#include <rviz/properties/status_property.h>

namespace rviz
{

ImuDisplay::ImuDisplay():
    fixed_frame_orientation_(true),
    box_enabled_(false),
    axes_enabled_(true),
    acc_enabled_(false),
    scene_node_(NULL),
    messages_received_(0)
{
    createProperties();

}

void ImuDisplay::onEnable()
{
    MessageFilterDisplay<sensor_msgs::Imu>::onEnable();

    if (box_enabled_)
        box_visual_->show();
    else
        box_visual_->hide();

    if (axes_enabled_)
        axes_visual_->show();
    else
        axes_visual_->hide();

    if (acc_enabled_)
        acc_visual_->show();
    else
        acc_visual_->hide();
}

void ImuDisplay::onDisable()
{
    MessageFilterDisplay<sensor_msgs::Imu>::onDisable();

    box_visual_->hide();
    axes_visual_->hide();
    acc_visual_->hide();
}

void ImuDisplay::onInitialize()
{
    MFDClass::onInitialize();

    // Make an Ogre::SceneNode to contain all our visuals.
    scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();

    // create orientation box visual
    box_visual_ = new ImuOrientationVisual(
                context_->getSceneManager(), scene_node_);

    // create orientation axes visual
    axes_visual_ = new ImuAxesVisual(
                context_->getSceneManager(), scene_node_);

    // create acceleration vector visual
    acc_visual_ = new ImuAccVisual(
                context_->getSceneManager(), scene_node_);
}

ImuDisplay::~ImuDisplay()
{
}

void ImuDisplay::reset()
{
    MFDClass::reset();
    messages_received_ = 0;
    setStatus(rviz::StatusProperty::Warn, "Topic", "No messages received" );

    box_visual_->hide();
    axes_visual_->hide();
    acc_visual_->hide();
}

void ImuDisplay::updateTop() {
    fixed_frame_orientation_ = fixed_frame_orientation_property_->getBool();
}

void ImuDisplay::updateBox() {
    box_enabled_ = box_enabled_property_->getBool();
    if (isEnabled() && box_enabled_)
        box_visual_->show();
    else
        box_visual_->hide();

    box_visual_->setScaleX(box_scale_x_property_->getFloat());
    box_visual_->setScaleY(box_scale_y_property_->getFloat());
    box_visual_->setScaleZ(box_scale_z_property_->getFloat());
    box_visual_->setColor(box_color_property_->getColor());
    box_visual_->setAlpha(box_alpha_property_->getFloat());
}

void ImuDisplay::updateAxes() {
    axes_enabled_ = axes_enabled_property_->getBool();
    if (isEnabled() && axes_enabled_)
        axes_visual_->show();
    else
        axes_visual_->hide();

    axes_visual_->setScale(axes_scale_property_->getFloat());

}

void ImuDisplay::updateAcc() {
    acc_enabled_ = acc_enabled_property_->getBool();
    if (isEnabled() && acc_enabled_)
        acc_visual_->show();
    else
        acc_visual_->hide();

    acc_visual_->setScale(acc_scale_property_->getFloat());
    acc_visual_->setColor(acc_color_property_->getColor());
    acc_visual_->setAlpha(acc_alpha_property_->getFloat());
    acc_visual_->setDerotated(acc_derotated_property_->getBool());
}

void ImuDisplay::processMessage( const sensor_msgs::Imu::ConstPtr& msg )
{
    if(!isEnabled())
        return;

    ++messages_received_;

    std::stringstream ss;
    ss << messages_received_ << " messages received";
    setStatus( rviz::StatusProperty::Ok, "Topic", ss.str().c_str() );

    Ogre::Quaternion orientation;
    Ogre::Vector3 position;
    if(!context_->getFrameManager()->getTransform(msg->header.frame_id,
                                                  msg->header.stamp,
                                                  position, orientation ))
    {
        ROS_ERROR("Error transforming from frame '%s' to frame '%s'",
                  msg->header.frame_id.c_str(), fixed_frame_.toStdString().c_str());
        return;
    }

    if (fixed_frame_orientation_) {
        /* Display IMU orientation is in the world-fixed frame. */
        Ogre::Vector3 unused;
        if(!context_->getFrameManager()->getTransform(context_->getFrameManager()->getFixedFrame(),
                                                      msg->header.stamp,
                                                      unused, orientation ))
        {
            ROS_ERROR("Error getting fixed frame transform");
            return;
        }
    }

    if (box_enabled_)
    {
        box_visual_->setMessage(msg);
        box_visual_->setFramePosition(position);
        box_visual_->setFrameOrientation(orientation);
        box_visual_->show();
    }
    if (axes_enabled_)
    {
        axes_visual_->setMessage(msg);
        axes_visual_->setFramePosition(position);
        axes_visual_->setFrameOrientation(orientation);
        axes_visual_->show();
    }
    if (acc_enabled_)
    {
        acc_visual_->setMessage(msg);
        acc_visual_->setFramePosition(position);
        acc_visual_->setFrameOrientation(orientation);
        acc_visual_->show();
    }
}

void ImuDisplay::createProperties()
{
    // **** top level properties
    fixed_frame_orientation_property_ = new rviz::BoolProperty("fixed_frame_orientation",
                                                   fixed_frame_orientation_,
                                                   "Use world fixed frame for display orientation instead of IMU reference frame",
                                                   this,
                                                   SLOT(updateTop()),
                                                   this);

    // **** box properties
    box_category_ = new rviz::Property("Box properties",
                                       QVariant(),
                                       "The list of all the box properties",
                                       this
                                       );
    box_enabled_property_ = new rviz::BoolProperty("Enable box",
                                                   box_enabled_,
                                                   "Enable the box display",
                                                   box_category_,
                                                   SLOT(updateBox()),
                                                   this);
    box_scale_x_property_ = new rviz::FloatProperty("x_scale",
                                                    1.0,
                                                    "Box length (x), in meters.",
                                                    box_category_,
                                                    SLOT(updateBox()),
                                                    this);
    box_scale_y_property_ = new rviz::FloatProperty("y_scale",
                                                    1.0,
                                                    "Box length (y), in meters.",
                                                    box_category_,
                                                    SLOT(updateBox()),
                                                    this);
    box_scale_z_property_ = new rviz::FloatProperty("z_scale",
                                                    1.0,
                                                    "Box length (z), in meters.",
                                                    box_category_,
                                                    SLOT(updateBox()),
                                                    this);
    box_color_property_  = new rviz::ColorProperty("Box color",
                                                   Qt::red,
                                                   "Color to draw IMU box",
                                                   box_category_,
                                                   SLOT(updateBox()),
                                                   this);
    box_alpha_property_ = new rviz::FloatProperty("Box alpha",
                                                  1.0,
                                                  "0 is fully transparent, 1.0 is fully opaque.",
                                                  box_category_,
                                                  SLOT(updateBox()),
                                                  this);

    // **** axes properties
    axes_category_ = new rviz::Property("Axes properties",
                                       QVariant(),
                                       "The list of all the axes properties",
                                       this
                                       );
    axes_enabled_property_ = new rviz::BoolProperty("Enable axes",
                                                   axes_enabled_,
                                                   "Enable the axes display",
                                                   axes_category_,
                                                   SLOT(updateAxes()),
                                                   this);
    axes_scale_property_ = new rviz::FloatProperty("Axes scale",
                                                   true,
                                                   "Axes size, in meters",
                                                   axes_category_,
                                                   SLOT(updateAxes()),
                                                   this);

    // **** acceleration vector properties
    acc_category_ = new rviz::Property("Acceleration properties",
                                       QVariant(),
                                       "The list of all the acceleration properties",
                                       this
                                       );
    acc_enabled_property_ = new rviz::BoolProperty("Enable acceleration",
                                                   acc_enabled_,
                                                   "Enable the acceleration display",
                                                   acc_category_,
                                                   SLOT(updateAcc()),
                                                   this);

    acc_derotated_property_ = new rviz::BoolProperty("Derotate acceleration",
                                                     true,
                                                     "If selected, the acceleration is derotated by the IMU orientation. Otherwise, the raw sensor reading is displayed.",
                                                     acc_category_,
                                                     SLOT(updateAcc()),
                                                     this);
    acc_scale_property_ = new rviz::FloatProperty("Acc. vector scale",
                                                  true,
                                                  "Acceleration vector size, in meters",
                                                  acc_category_,
                                                  SLOT(updateAcc()),
                                                  this);
    acc_color_property_ =  new rviz::ColorProperty("Acc. vector color",
                                                   Qt::red,
                                                   "Color to draw acceleration vector.",
                                                   acc_category_,
                                                   SLOT(updateAcc()),
                                                   this);
    acc_alpha_property_ = new rviz::FloatProperty("Acc. vector alpha",
                                                 1.0,
                                                 "0 is fully transparent, 1.0 is fully opaque.",
                                                 acc_category_,
                                                 SLOT(updateAcc()),
                                                  this);
}

} // end namespace rviz

// Tell pluginlib about this class.  It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz::ImuDisplay, rviz::Display)


