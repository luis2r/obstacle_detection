/*
 * Copyright (c) 2012, Willow Garage, Inc.
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

#include <sstream>

#include <tf/transform_listener.h>

#include "obstacles/mesh_resource_obstacle.h"
//#include "obstacles/pedestrian_mesh_resource_obstacle.h"
#include "obstacles/shape_obstacle.h"
//#include "obstacles/shape_marker.h"

#include "rviz/display_context.h"
#include "rviz/frame_manager.h"
#include "rviz/ogre_helpers/shape.h"
#include "rviz/properties/int_property.h"
#include "rviz/properties/property.h"
#include "rviz/properties/ros_topic_property.h"
#include "rviz/selection/selection_manager.h"
#include "rviz/validate_floats.h"

#include "rviz/ogre_helpers/billboard_line.h"

#include "obstacle_display.h"

using namespace rviz;

namespace obstacles_rviz_plugin {

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    ObstacleDisplay::ObstacleDisplay()
    : Display() {
        obstacle_topic_property_ = new RosTopicProperty("Obstacle Topic", "obstacle",
                QString::fromStdString(ros::message_traits::datatype<msgs_perception::Obstacle>()),
                "msgs_perception::Obstacle topic to subscribe to.  <topic>_array will also"
                " automatically be subscribed with type msgs_perception::ObstacleArray.",
                this, SLOT(updateTopic()));

        queue_size_property_ = new IntProperty("Queue Size", 100,
                "Advanced: set the size of the incoming Obstacle message queue.  Increasing this is"
                " useful if your incoming TF data is delayed significantly from your Obstacle data, "
                "but it can greatly increase memory usage if the messages are big.",
                this, SLOT(updateQueueSize()));
        queue_size_property_->setMin(0);

        namespaces_category_ = new Property("Namespaces", QVariant(), "", this);
    }

    void ObstacleDisplay::onInitialize() {
        tf_filter_ = new tf::MessageFilter<msgs_perception::Obstacle>(*context_->getTFClient(),
                fixed_frame_.toStdString(),
                queue_size_property_->getInt(),
                update_nh_);

        tf_filter_->connectInput(sub_);
        tf_filter_->registerCallback(boost::bind(&ObstacleDisplay::incomingObstacle, this, _1));
        tf_filter_->registerFailureCallback(boost::bind(&ObstacleDisplay::failedObstacle, this, _1, _2));
    }

    ObstacleDisplay::~ObstacleDisplay() {
        if (initialized()) {
            unsubscribe();

            clearObstacles();

            delete tf_filter_;
        }
    }

    void ObstacleDisplay::clearObstacles() {
        obstacles_.clear();
        obstacles_with_expiration_.clear();
        frame_locked_obstacles_.clear();
        tf_filter_->clear();
        namespaces_category_->removeChildren();
        namespaces_.clear();
    }

    void ObstacleDisplay::onEnable() {
        subscribe();
    }

    void ObstacleDisplay::onDisable() {
        unsubscribe();
        tf_filter_->clear();

        clearObstacles();
    }

    void ObstacleDisplay::updateQueueSize() {
        tf_filter_->setQueueSize((uint32_t) queue_size_property_->getInt());
    }

    void ObstacleDisplay::updateTopic() {
        unsubscribe();
        subscribe();
    }

    void ObstacleDisplay::subscribe() {
        if (!isEnabled()) {
            return;
        }

        std::string obstacle_topic = obstacle_topic_property_->getTopicStd();
        if (!obstacle_topic.empty()) {
            array_sub_.shutdown();
            sub_.unsubscribe();

            try {
                sub_.subscribe(update_nh_, obstacle_topic, queue_size_property_->getInt());
                array_sub_ = update_nh_.subscribe(obstacle_topic + "_array", queue_size_property_->getInt(), &ObstacleDisplay::incomingObstacleArray, this);
                setStatus(StatusProperty::Ok, "Topic", "OK");
            } catch (ros::Exception& e) {
                setStatus(StatusProperty::Error, "Topic", QString("Error subscribing: ") + e.what());
            }
        }
    }

    void ObstacleDisplay::unsubscribe() {
        sub_.unsubscribe();
        array_sub_.shutdown();
    }

    void ObstacleDisplay::deleteObstacle(ObstacleID id) {
        deleteObstacleStatus(id);

        M_IDToObstacle::iterator it = obstacles_.find(id);
        if (it != obstacles_.end()) {
            obstacles_with_expiration_.erase(it->second);
            frame_locked_obstacles_.erase(it->second);
            obstacles_.erase(it);
        }
    }

    void ObstacleDisplay::deleteObstaclesInNamespace(const std::string& ns) {
        std::vector<ObstacleID> to_delete;

        // TODO: this is inefficient, should store every in-use id per namespace and lookup by that
        M_IDToObstacle::iterator obstacle_it = obstacles_.begin();
        M_IDToObstacle::iterator obstacle_end = obstacles_.end();
        for (; obstacle_it != obstacle_end; ++obstacle_it) {
            if (obstacle_it->first.first == ns) {
                to_delete.push_back(obstacle_it->first);
            }
        }

        {
            std::vector<ObstacleID>::iterator it = to_delete.begin();
            std::vector<ObstacleID>::iterator end = to_delete.end();
            for (; it != end; ++it) {
                deleteObstacle(*it);
            }
        }
    }

    void ObstacleDisplay::setObstacleStatus(ObstacleID id, StatusLevel level, const std::string& text) {
        std::stringstream ss;
        ss << id.first << "/" << id.second;
        std::string obstacle_name = ss.str();
        setStatusStd(level, obstacle_name, text);
    }

    void ObstacleDisplay::deleteObstacleStatus(ObstacleID id) {
        std::stringstream ss;
        ss << id.first << "/" << id.second;
        std::string obstacle_name = ss.str();
        deleteStatusStd(obstacle_name);
    }

    void ObstacleDisplay::incomingObstacleArray(const msgs_perception::ObstacleArray::ConstPtr& array) {
        std::vector<msgs_perception::Obstacle>::const_iterator it = array->obstacle.begin();
        std::vector<msgs_perception::Obstacle>::const_iterator end = array->obstacle.end();
        for (; it != end; ++it) {
            const msgs_perception::Obstacle& obstacle = *it;
            tf_filter_->add(msgs_perception::Obstacle::Ptr(new msgs_perception::Obstacle(obstacle)));
        }
    }

    void ObstacleDisplay::incomingObstacle(const msgs_perception::Obstacle::ConstPtr& obstacle) {
        boost::mutex::scoped_lock lock(queue_mutex_);

        message_queue_.push_back(obstacle);
    }

    void ObstacleDisplay::failedObstacle(const ros::MessageEvent<msgs_perception::Obstacle>& obstacle_evt, tf::FilterFailureReason reason) {
        msgs_perception::Obstacle::ConstPtr obstacle = obstacle_evt.getConstMessage();
        std::string authority = obstacle_evt.getPublisherName();
        std::string error = context_->getFrameManager()->discoverFailureReason(obstacle->header.frame_id, obstacle->header.stamp, authority, reason);
        setObstacleStatus(ObstacleID(obstacle->ns, obstacle->id), StatusProperty::Error, error);
    }

    bool validateFloats(const msgs_perception::Obstacle& msg) {
        bool valid = true;
        valid = valid && rviz::validateFloats(msg.pose);
        valid = valid && rviz::validateFloats(msg.scale);
                valid = valid && rviz::validateFloats(msg.color);
        return valid;
    }

    void ObstacleDisplay::processMessage(const msgs_perception::Obstacle::ConstPtr& message) {
        if (!validateFloats(*message)) {
            setObstacleStatus(ObstacleID(message->ns, message->id), StatusProperty::Error, "Contains invalid floating point values (nans or infs)");
            return;
        }

        switch (message->action) {
            case msgs_perception::Obstacle::ADD:
                processAdd(message);
                break;

            case msgs_perception::Obstacle::DELETE:
                processDelete(message);
                break;

            default:
                ROS_ERROR("Unknown obstacle action: %d\n", message->action);
        }
    }

    void ObstacleDisplay::processAdd(const msgs_perception::Obstacle::ConstPtr& message) {
        QString namespace_name = QString::fromStdString(message->ns);
        M_Namespace::iterator ns_it = namespaces_.find(namespace_name);
        if (ns_it == namespaces_.end()) {
            ns_it = namespaces_.insert(namespace_name, new ObstacleNamespace(namespace_name, namespaces_category_, this));
        }

        if (!ns_it.value()->isEnabled()) {
            return;
        }

        deleteObstacleStatus(ObstacleID(message->ns, message->id));

        bool create = true;
        ObstacleBasePtr obstacle;

        M_IDToObstacle::iterator it = obstacles_.find(ObstacleID(message->ns, message->id));
        if (it != obstacles_.end()) {
            obstacle = it->second;
            obstacles_with_expiration_.erase(obstacle);
//                        ROS_INFO_STREAM("Entra: " << 22);
                        if (message->type == obstacle->getMessage()->type) {
                            create = false;
                        } else {
            obstacles_.erase(it);////no estaba comentado
                        }
        }

//        if (message->track_status == 0 || (message->pose.position.x == 0 && message->pose.position.y == 0 && message->pose.position.z == 0) ) //= no target
//        {
//            create = false;
//        }
            

        if (create) {
            //            ROS_INFO_STREAM("Entra: " << 22);
            //ROS_INFO("x%f\n", message->pose.position.x);
            switch (message->type) {

                    //                case msgs_perception::Obstacle::CUBE:
                case -1://////-1 = unknow object
                {
                    //                    obstacle.reset(new ShapeObstacle(this, context_, scene_node_));
                    obstacle.reset(new ShapeObstacle(this, context_, scene_node_));
                }
                    break;
                    //case msgs_perception::Obstacle::CYLINDER:
                    //                case msgs_perception::Obstacle::SPHERE:
                    //                {
                    //                    //                    obstacle.reset(new ShapeObstacle(this, context_, scene_node_));
                    //                    obstacle.reset(new PedestrianMeshResourceObstacle(this, context_, scene_node_));
                    //                }
                    //                    break;
                    //                case msgs_perception::Obstacle::MESH_RESOURCE:
                case 0://////0 = pedestrian
                {
                    //                    obstacle.reset(new PedestrianMeshResourceObstacle(this, context_, scene_node_));
                    obstacle.reset(new MeshResourceObstacle(this, context_, scene_node_));
                }
                    break;
                case 1://////1 = vehicle
                {
                    obstacle.reset(new MeshResourceObstacle(this, context_, scene_node_));
                }
                    break;
                case 2://// 2 = bicycle 
                {
                    obstacle.reset(new ShapeObstacle(this, context_, scene_node_));
                }
                    break;
                case 3://// 3 = traffic light
                {
                    obstacle.reset(new ShapeObstacle(this, context_, scene_node_));
                }
                    break;
                case 4: //// 4 = traffic sign
                {
                    obstacle.reset(new ShapeObstacle(this, context_, scene_node_));
                }
                    break;

                default:
                    ROS_ERROR("Unknown obstacle type: %d", message->type);
            }

            obstacles_.insert(std::make_pair(ObstacleID(message->ns, message->id), obstacle));
        }

        if (obstacle) {
            obstacle->setMessage(message);

            if (message->lifetime.toSec() > 0.0001f) {
                obstacles_with_expiration_.insert(obstacle);
            }

            if (message->frame_locked) {
                frame_locked_obstacles_.insert(obstacle);
            }

            context_->queueRender();
        }
    }

    void ObstacleDisplay::processDelete(const msgs_perception::Obstacle::ConstPtr& message) {
        deleteObstacle(ObstacleID(message->ns, message->id));
        context_->queueRender();
    }

    void ObstacleDisplay::update(float wall_dt, float ros_dt) {
        V_ObstacleMessage local_queue;

        {
            boost::mutex::scoped_lock lock(queue_mutex_);

            local_queue.swap(message_queue_);
        }

        if (!local_queue.empty()) {
            V_ObstacleMessage::iterator message_it = local_queue.begin();
            V_ObstacleMessage::iterator message_end = local_queue.end();
            for (; message_it != message_end; ++message_it) {
                msgs_perception::Obstacle::ConstPtr& obstacle = *message_it;

                processMessage(obstacle);
            }
        }

        {
            S_ObstacleBase::iterator it = obstacles_with_expiration_.begin();
            S_ObstacleBase::iterator end = obstacles_with_expiration_.end();
            for (; it != end;) {
                ObstacleBasePtr obstacle = *it;
                if (obstacle->expired()) {
                    ++it;
                    deleteObstacle(obstacle->getID());
                } else {
                    ++it;
                }
            }
        }

        {
            S_ObstacleBase::iterator it = frame_locked_obstacles_.begin();
            S_ObstacleBase::iterator end = frame_locked_obstacles_.end();
            for (; it != end; ++it) {
                ObstacleBasePtr obstacle = *it;
                obstacle->updateFrameLocked();
            }
        }

        // Update animation states
        {
            for (M_IDToObstacle::iterator it = obstacles_.begin(); it != obstacles_.end(); ++it) {
                it->second->update(ros_dt);
            }
        }
    }

    void ObstacleDisplay::fixedFrameChanged() {
        tf_filter_->setTargetFrame(fixed_frame_.toStdString());

        clearObstacles();
    }

    void ObstacleDisplay::reset() {
        Display::reset();
        clearObstacles();
    }

    /////////////////////////////////////////////////////////////////////////////////
    // ObstacleNamespace

    ObstacleNamespace::ObstacleNamespace(const QString& name, Property* parent_property, ObstacleDisplay* owner)
    : BoolProperty(name, true,
    "Enable/disable all obstacles in this namespace.",
    parent_property)
    , owner_(owner) {
        // Can't do this connect in chained constructor above because at
        // that point it doesn't really know that "this" is a
        // ObstacleNamespace*, so the signal doesn't get connected.
        connect(this, SIGNAL(changed()), this, SLOT(onEnableChanged()));
    }

    void ObstacleNamespace::onEnableChanged() {
        if (!isEnabled()) {
            owner_->deleteObstaclesInNamespace(getName().toStdString());
        }
    }

} // namespace obstacle_rviz_plugin

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(obstacles_rviz_plugin::ObstacleDisplay, rviz::Display)
