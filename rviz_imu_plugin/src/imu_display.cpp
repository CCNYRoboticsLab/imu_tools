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

#include "rviz_imu_plugin/imu_display.h"

namespace rviz
{

ImuDisplay::ImuDisplay(): 
  Display(),
  box_enabled_(true),
  axes_enabled_(true),
  acc_enabled_(false),
  scene_node_(NULL),
  messages_received_(0)
{

}

void ImuDisplay::onInitialize()
{
  // Make an Ogre::SceneNode to contain all our visuals.
  scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
  
  // create orientation box visual 
  box_visual_ = new ImuOrientationVisual(
    vis_manager_->getSceneManager(), scene_node_);

  // create orientation axes visual 
  axes_visual_ = new ImuAxesVisual(
    vis_manager_->getSceneManager(), scene_node_);

  // create acceleration vector visual 
  acc_visual_ = new ImuAccVisual(
    vis_manager_->getSceneManager(), scene_node_);

  tf_filter_ =
    new tf::MessageFilter<sensor_msgs::Imu>( *vis_manager_->getTFClient(),
                                             "", 100, update_nh_ );
  tf_filter_->connectInput( sub_ );
  tf_filter_->registerCallback( boost::bind( &ImuDisplay::incomingMessage,
                                             this, _1 ));

  vis_manager_->getFrameManager()
    ->registerFilterForTransformStatusCheck( tf_filter_, this );
}

ImuDisplay::~ImuDisplay()
{
  unsubscribe();
  clear();
  delete tf_filter_;
}

void ImuDisplay::clear()
{
  tf_filter_->clear();
  messages_received_ = 0;
  setStatus(rviz::status_levels::Warn, "Topic", "No messages received" );

  box_visual_->hide();
  axes_visual_->hide();
  acc_visual_->hide();
}

void ImuDisplay::setTopic( const std::string& topic )
{
  unsubscribe();
  clear();
  topic_ = topic;
  subscribe();

  propertyChanged(topic_property_);
  causeRender();
}

void ImuDisplay::setBoxEnabled(bool enabled)
{
  box_enabled_ = enabled;
  if (enabled) box_visual_->show();
  else         box_visual_->hide();
  propertyChanged(box_enabled_property_);
  causeRender();
}

void ImuDisplay::setAxesEnabled(bool enabled)
{
  axes_enabled_ = enabled;
  if (enabled) axes_visual_->show();
  else         axes_visual_->hide();
  propertyChanged(box_enabled_property_);
  causeRender();
}

void ImuDisplay::setAccEnabled(bool enabled)
{
  acc_enabled_ = enabled;
  if (enabled) acc_visual_->show();
  else         acc_visual_->hide();
  propertyChanged(acc_enabled_property_);
  causeRender();
}

void ImuDisplay::setBoxScaleX(float x)
{
  box_visual_->setScaleX(x);
  propertyChanged(box_scale_x_property_);
  causeRender();
}

void ImuDisplay::setBoxScaleY(float y)
{
  box_visual_->setScaleY(y);
  propertyChanged(box_scale_y_property_);
  causeRender();
}

void ImuDisplay::setBoxScaleZ(float z)
{
  box_visual_->setScaleZ(z);
  propertyChanged(box_scale_z_property_);
  causeRender();
}

void ImuDisplay::setBoxColor(const rviz::Color& color)
{
  box_visual_->setColor(color);
  propertyChanged(box_color_property_);
  causeRender();
}

void ImuDisplay::setBoxAlpha(float alpha)
{
  box_visual_->setAlpha(alpha);
  propertyChanged(box_alpha_property_);
  causeRender();
}

void ImuDisplay::setAxesScale(float scale)
{
  axes_visual_->setScale(scale);
  propertyChanged(axes_scale_property_);
  causeRender();
}


void ImuDisplay::setAccScale(float scale)
{
  acc_visual_->setScale(scale);
  propertyChanged(acc_scale_property_);
  causeRender();
}

void ImuDisplay::setAccColor(const rviz::Color& color)
{
  acc_visual_->setColor(color);
  propertyChanged(acc_color_property_);
  causeRender();
}

void ImuDisplay::setAccAlpha(float alpha)
{
  acc_visual_->setAlpha(alpha);
  propertyChanged(acc_alpha_property_);
  causeRender();
}

void ImuDisplay::setAccDerotated(bool derotated)
{
  acc_visual_->setDerotated(derotated);
  propertyChanged(acc_derotated_property_);
  causeRender();
}

void ImuDisplay::subscribe()
{
  // If we are not actually enabled, don't do it.
  if (!isEnabled()) return;

  try
  {
    sub_.subscribe( update_nh_, topic_, 10 );
    setStatus( rviz::status_levels::Ok, "Topic", "OK" );
  }
  catch( ros::Exception& e )
  {
    setStatus( rviz::status_levels::Error, "Topic",
               std::string( "Error subscribing: " ) + e.what() );
  }
}

void ImuDisplay::unsubscribe()
{
  sub_.unsubscribe();
}

void ImuDisplay::onEnable()
{
  subscribe();
}

void ImuDisplay::onDisable()
{
  unsubscribe();
  clear();
}

void ImuDisplay::fixedFrameChanged()
{
  tf_filter_->setTargetFrame(fixed_frame_);
  clear();
}

void ImuDisplay::incomingMessage( const sensor_msgs::Imu::ConstPtr& msg )
{
  ++messages_received_;
  
  std::stringstream ss;
  ss << messages_received_ << " messages received";
  setStatus( rviz::status_levels::Ok, "Topic", ss.str() );

  Ogre::Quaternion orientation;
  Ogre::Vector3 position;
  if(!vis_manager_->getFrameManager()->getTransform(msg->header.frame_id,
                                                    msg->header.stamp,
                                                    position, orientation ))
  {
    ROS_DEBUG("Error transforming from frame '%s' to frame '%s'",
               msg->header.frame_id.c_str(), fixed_frame_.c_str());
    return;
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

void ImuDisplay::reset()
{
  Display::reset();
  clear();
}

void ImuDisplay::createProperties()
{
  // **** topic properties

  topic_property_ =
    property_manager_->createProperty<rviz::ROSTopicStringProperty>(
      "Topic",
       property_prefix_,
       boost::bind( &ImuDisplay::getTopic, this ),
       boost::bind( &ImuDisplay::setTopic, this, _1 ),
       parent_category_,
       this );

  setPropertyHelpText( topic_property_, "sensor_msgs::Imu topic to subscribe to." );

  rviz::ROSTopicStringPropertyPtr topic_prop = topic_property_.lock();
  topic_prop->setMessageType( ros::message_traits::datatype<sensor_msgs::Imu>() );

  // **** box properties

  box_category_ = 
    property_manager_->createCategory("Orientation box", 
      property_prefix_, parent_category_, this);

  box_enabled_property_ = 
    property_manager_->createProperty<BoolProperty>( 
      "Enable box", 
      property_prefix_, 
      boost::bind(&ImuDisplay::getBoxEnabled, this),
      boost::bind(&ImuDisplay::setBoxEnabled, this, _1), 
      box_category_, 
      this);

  setPropertyHelpText(box_enabled_property_, "Enable the box display");
   
  box_scale_x_property_ = 
    property_manager_->createProperty<FloatProperty>( 
      "x scale", 
      property_prefix_, 
      boost::bind(&ImuDisplay::getBoxScaleX, this),
      boost::bind(&ImuDisplay::setBoxScaleX, this, _1), 
      box_category_, 
      this);

  setPropertyHelpText(box_scale_x_property_, "Box length (x), in meters.");
   
  box_scale_y_property_ = 
    property_manager_->createProperty<FloatProperty>( 
      "y scale", 
      property_prefix_, 
      boost::bind(&ImuDisplay::getBoxScaleY, this),
      boost::bind(&ImuDisplay::setBoxScaleY, this, _1), 
      box_category_, 
      this);

  setPropertyHelpText(box_scale_y_property_, "Box width (y), in meters.");
   
  box_scale_z_property_ = 
    property_manager_->createProperty<FloatProperty>( 
      "z scale", 
      property_prefix_, 
      boost::bind(&ImuDisplay::getBoxScaleZ, this),
      boost::bind(&ImuDisplay::setBoxScaleZ, this, _1), 
      box_category_, 
      this);

  setPropertyHelpText(box_scale_z_property_, "Box height (z), in meters.");
  
  box_color_property_ =
    property_manager_->createProperty<rviz::ColorProperty>( 
      "Box color",
      property_prefix_,
      boost::bind( &ImuDisplay::getBoxColor, this),
      boost::bind( &ImuDisplay::setBoxColor, this, _1),
      box_category_,
      this );

  setPropertyHelpText(box_color_property_, "Color to draw IMU box.");

  box_alpha_property_ =
    property_manager_->createProperty<rviz::FloatProperty>( 
      "Box alpha",
      property_prefix_,
      boost::bind(&ImuDisplay::getBoxAlpha, this),
      boost::bind(&ImuDisplay::setBoxAlpha, this, _1),
      box_category_,
      this );
 
  setPropertyHelpText(box_alpha_property_, "0 is fully transparent, 1.0 is fully opaque.");

  // **** axes properties

  axes_category_ = 
    property_manager_->createCategory("Orientation axes", 
      property_prefix_, parent_category_, this);

  axes_enabled_property_ = 
    property_manager_->createProperty<BoolProperty>( 
      "Enable axes", 
      property_prefix_, 
      boost::bind(&ImuDisplay::getAxesEnabled, this),
      boost::bind(&ImuDisplay::setAxesEnabled, this, _1), 
      axes_category_, 
      this);

  setPropertyHelpText(axes_enabled_property_, "Enable the axes display");

  axes_scale_property_ = 
    property_manager_->createProperty<FloatProperty>( 
      "Axes scale", 
      property_prefix_, 
      boost::bind(&ImuDisplay::getAxesScale, this),
      boost::bind(&ImuDisplay::setAxesScale, this, _1), 
      axes_category_,
      this);
  
  setPropertyHelpText(axes_scale_property_, "Axes size, in meters");

  // **** acceleration vector properties

  acc_category_ = 
    property_manager_->createCategory("Acceleration vector", 
      property_prefix_, parent_category_, this);

  acc_enabled_property_ = 
    property_manager_->createProperty<BoolProperty>( 
      "Enable acceleration", 
      property_prefix_, 
      boost::bind(&ImuDisplay::getAccEnabled, this),
      boost::bind(&ImuDisplay::setAccEnabled, this, _1), 
      acc_category_, 
      this);

  setPropertyHelpText(axes_enabled_property_, "Enable the acceleration display");

  acc_derotated_property_ = 
    property_manager_->createProperty<BoolProperty>( 
      "Derotate acceleration", 
      property_prefix_, 
      boost::bind(&ImuDisplay::getAccDerotated, this),
      boost::bind(&ImuDisplay::setAccDerotated, this, _1), 
      acc_category_, 
      this);

  setPropertyHelpText(axes_scale_property_, "If selected, the acceleration is derotated by the IMU orientation. Otherwise, the raw sensor reading is displayed.");

  acc_scale_property_ = 
    property_manager_->createProperty<FloatProperty>( 
      "Acc. vector scale", 
      property_prefix_, 
      boost::bind(&ImuDisplay::getAccScale, this),
      boost::bind(&ImuDisplay::setAccScale, this, _1), 
      acc_category_,
      this);
  
  setPropertyHelpText(axes_scale_property_, "Acceleration vector size, in meters");

  acc_color_property_ =
    property_manager_->createProperty<rviz::ColorProperty>( 
      "Acc. vector color",
      property_prefix_,
      boost::bind( &ImuDisplay::getAccColor, this),
      boost::bind( &ImuDisplay::setAccColor, this, _1),
      acc_category_,
      this );

  setPropertyHelpText(box_color_property_, "Color to draw acceleration vector.");

  box_alpha_property_ =
    property_manager_->createProperty<rviz::FloatProperty>( 
      "Acc. vector alpha",
      property_prefix_,
      boost::bind(&ImuDisplay::getAccAlpha, this),
      boost::bind(&ImuDisplay::setAccAlpha, this, _1),
      acc_category_,
      this );
}

} // end namespace rviz

// Tell pluginlib about this class.  It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS(rviz_imu_plugin, Imu, rviz::ImuDisplay, rviz::Display)


