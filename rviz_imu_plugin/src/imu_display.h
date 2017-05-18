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

#ifndef RVIZ_IMU_PLUGIN_IMU_DISPLAY_H
#define RVIZ_IMU_PLUGIN_IMU_DISPLAY_H

#include <message_filters/subscriber.h>
#include <tf/message_filter.h>
#include <sensor_msgs/Imu.h>
#include <rviz/display.h>
#include <rviz/visualization_manager.h>
#include <rviz/properties/property.h>
#include <rviz/frame_manager.h>
#include <rviz/message_filter_display.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <tf/transform_listener.h>

#include <rviz/properties/bool_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/ros_topic_property.h>
#include <rviz/display_group.h>

#include "imu_axes_visual.h"
#include "imu_orientation_visual.h"
#include "imu_acc_visual.h"

namespace Ogre
{
class SceneNode;
}

namespace rviz
{

class ImuDisplay: public rviz::MessageFilterDisplay<sensor_msgs::Imu>
{
    Q_OBJECT
public:

    ImuDisplay();
    virtual ~ImuDisplay();

    virtual void onInitialize();
    virtual void reset();
    virtual void createProperties();

    void setTopic(const std::string& topic);
    const std::string& getTopic() { return topic_; }

    void setBoxEnabled(bool enabled);
    void setBoxScaleX(float x);
    void setBoxScaleY(float y);
    void setBoxScaleZ(float z);
    void setBoxColor(const Color& color);
    void setBoxAlpha(float alpha);

    void setAxesEnabled(bool enabled);
    void setAxesScale(float scale);

    void setAccEnabled(bool enabled);
    void setAccDerotated(bool derotated);
    void setAccScale(float scale);
    void setAccColor(const Color& color);
    void setAccAlpha(float alpha);

    bool  getBoxEnabled() { return box_enabled_; }
    float getBoxScaleX()  { return box_visual_->getScaleX(); }
    float getBoxScaleY()  { return box_visual_->getScaleY(); }
    float getBoxScaleZ()  { return box_visual_->getScaleZ(); }
    float getBoxAlpha()   { return box_visual_->getAlpha(); }
    const QColor& getBoxColor() { return box_visual_->getColor(); }

    bool  getAxesEnabled() { return axes_enabled_; }
    float getAxesScale()   { return axes_visual_->getScale(); }

    bool  getAccEnabled()   { return acc_enabled_; }
    float getAccDerotated() { return acc_visual_->getDerotated(); }
    float getAccScale()     { return acc_visual_->getScale(); }
    float getAccAlpha()     { return acc_visual_->getAlpha(); }
    const QColor& getAccColor() { return acc_visual_->getColor(); }

protected Q_SLOTS:

    void updateTop();
    void updateBox();
    void updateAxes();
    void updateAcc();

private:

    // Property objects for user-editable properties.
    rviz::BoolProperty*  fixed_frame_orientation_property_;

    rviz::Property* box_category_;
    rviz::Property* axes_category_;
    rviz::Property* acc_category_;

//    rviz::RosTopicProperty* topic_property_;
    rviz::BoolProperty*  box_enabled_property_;
    rviz::FloatProperty* box_scale_x_property_;
    rviz::FloatProperty* box_scale_y_property_;
    rviz::FloatProperty* box_scale_z_property_;
    rviz::ColorProperty* box_color_property_;
    rviz::FloatProperty* box_alpha_property_;

    rviz::BoolProperty*  axes_enabled_property_;
    rviz::FloatProperty* axes_scale_property_;

    rviz::BoolProperty*  acc_enabled_property_;
    rviz::BoolProperty*  acc_derotated_property_;
    rviz::FloatProperty* acc_scale_property_;
    rviz::ColorProperty* acc_color_property_;
    rviz::FloatProperty* acc_alpha_property_;

    // Differetn types of visuals
    ImuOrientationVisual * box_visual_;
    ImuAxesVisual        * axes_visual_;
    ImuAccVisual         * acc_visual_;

    // User-editable property variables.
    std::string topic_;
    bool fixed_frame_orientation_;
    bool box_enabled_;
    bool axes_enabled_;
    bool acc_enabled_;

    // A node in the Ogre scene tree to be the parent of all our visuals.
    Ogre::SceneNode* scene_node_;

    int messages_received_;

    // Function to handle an incoming ROS message.
    void processMessage( const sensor_msgs::Imu::ConstPtr& msg);

};

} // end namespace rviz

#endif // IMU_DISPLAY_H

