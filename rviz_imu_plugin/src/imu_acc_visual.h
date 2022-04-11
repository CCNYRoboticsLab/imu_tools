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
#ifndef RVIZ_IMU_PLUGIN_IMU_ACC_VISUAL_H
#define RVIZ_IMU_PLUGIN_IMU_ACC_VISUAL_H

#include <sensor_msgs/msg/imu.hpp>
#include <rviz_common/display.hpp>
#include <QColor>

namespace rviz_rendering {
class Arrow;
}

namespace rviz_imu_plugin {

class ImuAccVisual
{
  public:
    // Constructor.  Creates the visual stuff and puts it into the
    // scene, but in an unconfigured state.
    ImuAccVisual(Ogre::SceneManager* scene_manager,
                 Ogre::SceneNode* parent_node);

    // Destructor.  Removes the visual stuff from the scene.
    virtual ~ImuAccVisual();

    // Configure the visual to show the data in the message.
    void setMessage(const sensor_msgs::msg::Imu::ConstSharedPtr msg);

    // Set the pose of the coordinate frame the message refers to.
    // These could be done inside setMessage(), but that would require
    // calls to FrameManager and error handling inside setMessage(),
    // which doesn't seem as clean.  This way ImuVisual is only
    // responsible for visualization.
    void setFramePosition(const Ogre::Vector3& position);
    void setFrameOrientation(const Ogre::Quaternion& orientation);

    // Set the color and alpha of the visual, which are user-editable

    void setScale(float scale);
    void setColor(const QColor& color);
    void setAlpha(float alpha);
    void setDerotated(bool derotated);

    float getScale()
    {
        return scale_;
    }
    const QColor& getColor()
    {
        return color_;
    }
    float getAlpha()
    {
        return alpha_;
    }
    bool getDerotated()
    {
        return derotated_;
    }

    void show();
    void hide();

  private:
    void create();

    rviz_rendering::Arrow* acc_vector_;

    Ogre::Vector3 direction_;  // computed from IMU message

    float arrow_length_;  // computed from IMU message
    float arrow_radius_;
    float head_length_;
    float head_radius_;

    float scale_;
    float alpha_;
    QColor color_;

    bool derotated_;

    // A SceneNode whose pose is set to match the coordinate frame of
    // the Imu message header.
    Ogre::SceneNode* frame_node_;

    // The SceneManager, kept here only so the destructor can ask it to
    // destroy the ``frame_node_``.
    Ogre::SceneManager* scene_manager_;
};

}  // namespace rviz_imu_plugin

#endif  // RVIZ_IMU_PLUGIN_IMU_ORIENTATATION_VISUAL_H
