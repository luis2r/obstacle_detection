/*
 * Copyright (c) 2011, Willow Garage, Inc.
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

#include "rviz/properties/ros_topic_property.h"
#include "rviz/properties/int_property.h"

#include "obstacle_array_display.h"

using namespace rviz;

namespace obstacles_rviz_plugin
{

ObstacleArrayDisplay::ObstacleArrayDisplay()
  : ObstacleDisplay()
{
  obstacle_topic_property_->setMessageType( QString::fromStdString( ros::message_traits::datatype<msgs_perception::ObstacleArray>() ));
  obstacle_topic_property_->setValue( "visualization_obstacle_array" );
  obstacle_topic_property_->setDescription( "msgs_perception::ObstacleArray topic to subscribe to." );

  queue_size_property_->setDescription( "Advanced: set the size of the incoming Obstacle message queue. "
                                        " This should generally be at least a few times larger than the number of Obstacles in each ObstacleArray." );
}

void ObstacleArrayDisplay::subscribe()
{
  if ( !isEnabled() )
  {
    return;
  }

  std::string topic = obstacle_topic_property_->getTopicStd();
  if( !topic.empty() )
  {
    array_sub_.shutdown();

    try
    {
      array_sub_ = update_nh_.subscribe( topic, queue_size_property_->getInt(), &ObstacleArrayDisplay::handleObstacleArray, this );
      setStatus( StatusProperty::Ok, "Topic", "OK" );
    }
    catch( ros::Exception& e )
    {
      setStatus( StatusProperty::Error, "Topic", QString( "Error subscribing: " ) + e.what() );
    }
  }
}

void ObstacleArrayDisplay::unsubscribe()
{
  array_sub_.shutdown();
}

// I seem to need this wrapper function to make the compiler like my
// function pointer in the .subscribe() call above.
void ObstacleArrayDisplay::handleObstacleArray( const msgs_perception::ObstacleArray::ConstPtr& array )
{
  incomingObstacleArray( array );
}

} // end namespace obstacle_rviz_plugin

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( obstacles_rviz_plugin::ObstacleArrayDisplay, rviz::Display )
