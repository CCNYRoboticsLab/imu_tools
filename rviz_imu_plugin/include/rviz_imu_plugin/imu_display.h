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
#include <rviz/properties/property_manager.h>
#include <rviz/frame_manager.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <tf/transform_listener.h>

#include "rviz_imu_plugin/imu_axes_visual.h"
#include "rviz_imu_plugin/imu_orientation_visual.h"
#include "rviz_imu_plugin/imu_acc_visual.h"

namespace Ogre
{
class SceneNode;
}

namespace rviz
{

class ImuDisplay: public rviz::Display
{
  public:

    ImuDisplay();
    virtual ~ImuDisplay();

    virtual void onInitialize();
    virtual void fixedFrameChanged();
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
    const Color& getBoxColor() { return box_visual_->getColor(); }

    bool  getAxesEnabled() { return axes_enabled_; }
    float getAxesScale()   { return axes_visual_->getScale(); }

    bool  getAccEnabled()   { return acc_enabled_; }
    float getAccDerotated() { return acc_visual_->getDerotated(); }
    float getAccScale()     { return acc_visual_->getScale(); }
    float getAccAlpha()     { return acc_visual_->getAlpha(); }
    const Color& getAccColor() { return acc_visual_->getColor(); }

  protected:

    virtual void onEnable();
    virtual void onDisable();

  private:

    // Differetn types of visuals
    ImuOrientationVisual * box_visual_;
    ImuAxesVisual        * axes_visual_;
    ImuAccVisual         * acc_visual_;

    // User-editable property variables.
    std::string topic_;
    bool box_enabled_;
    bool axes_enabled_;
    bool acc_enabled_;

    // A node in the Ogre scene tree to be the parent of all our visuals.
    Ogre::SceneNode* scene_node_;

    // Data input: Subscriber and tf message filter.
    message_filters::Subscriber<sensor_msgs::Imu> sub_;
    tf::MessageFilter<sensor_msgs::Imu>* tf_filter_;
    int messages_received_;

    // Property objects for user-editable properties.
    rviz::ROSTopicStringPropertyWPtr topic_property_;

    CategoryPropertyWPtr box_category_;
    CategoryPropertyWPtr axes_category_;
    CategoryPropertyWPtr acc_category_;

    BoolPropertyWPtr  box_enabled_property_;
    FloatPropertyWPtr box_scale_x_property_;
    FloatPropertyWPtr box_scale_y_property_;
    FloatPropertyWPtr box_scale_z_property_;
    ColorPropertyWPtr box_color_property_;
    FloatPropertyWPtr box_alpha_property_;

    BoolPropertyWPtr  axes_enabled_property_;
    FloatPropertyWPtr axes_scale_property_;

    BoolPropertyWPtr  acc_enabled_property_;
    BoolPropertyWPtr  acc_derotated_property_;
    FloatPropertyWPtr acc_scale_property_;
    ColorPropertyWPtr acc_color_property_;
    FloatPropertyWPtr acc_alpha_property_;

    // Function to handle an incoming ROS message.
    void incomingMessage( const sensor_msgs::Imu::ConstPtr& msg);

    // Internal helpers which do the work of subscribing and
    // unsubscribing from the ROS topic.
    void subscribe();
    void unsubscribe();

    // A helper to clear this display back to the initial state.
    void clear();
};

} // end namespace rviz

#endif // IMU_DISPLAY_H

