/*
 * Copyright (c) 2009, Willow Garage, Inc.
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

#include "shape_obstacle.h"
#include "obstacle_selection_handler.h"
#include "../obstacle_display.h"

#include "rviz/display_context.h"
#include "rviz/selection/selection_manager.h"

#include <rviz/ogre_helpers/shape.h>

#include <OgreSceneNode.h>
#include <OgreMatrix3.h>

#include <cmath>

namespace obstacles_rviz_plugin {

    ShapeObstacle::ShapeObstacle(ObstacleDisplay* owner, DisplayContext* context, Ogre::SceneNode* parent_node)
    : ObstacleBase(owner, context, parent_node)
    , shape_(0) {
    }

    ShapeObstacle::~ShapeObstacle() {
        delete shape_;
    }

    void ShapeObstacle::onNewMessage(const ObstacleConstPtr& old_message,
            const ObstacleConstPtr& new_message) {
        //        if (!shape_ /*|| old_message->type != new_message->type*/) {

        delete shape_;
        shape_ = 0;

        Shape::Type shape_type = Shape::Cube;
        switch (new_message->type) {
            case -1: shape_type = Shape::Cube;
                break;
            case 2: shape_type = Shape::Cylinder;
                break;
            case 3: shape_type = Shape::Sphere;
                break;
            case 4: shape_type = Shape::Sphere;
                break;
            default:
                ROS_BREAK();
                break;
        }
        shape_ = new Shape(shape_type, context_->getSceneManager(), scene_node_);

        handler_.reset(new ObstacleSelectionHandler(this, ObstacleID(new_message->ns, new_message->id), context_));
        handler_->addTrackedObjects(shape_->getRootNode());
        //        }

        Ogre::Vector3 pos, scale, scale_correct;
        Ogre::Quaternion orient;
        transform(new_message, pos, orient, scale);

        if (owner_ && (new_message->scale.x * new_message->scale.y
                * new_message->scale.z == 0.0f)) {
            owner_->setObstacleStatus(getID(), StatusProperty::Warn,
                    "Scale of 0 in one of x/y/z");
        }

        setPosition(pos);
        setOrientation(orient * Ogre::Quaternion(Ogre::Degree(90), Ogre::Vector3(1, 0, 0)));
        
                if (new_message->track_status == 0 || (new_message->pose.position.x == 0 && new_message->pose.position.y == 0 && new_message->pose.position.z == 0) ) //= no target
        {
            scale = Ogre::Vector3(0.01, 0.01, 0.01);
        }

        scale_correct = Ogre::Quaternion(Ogre::Degree(90), Ogre::Vector3(1, 0, 0)) * scale;

        shape_->setScale(scale_correct);



        //        ROS_INFO_STREAM("color: " << new_message->color);



        float v = 100 - sqrt(pow(new_message->pose.position.x, 2) + pow(new_message->pose.position.y, 2))/*new_message->range_track*/;
        //        float v = new_message->range_track;
        float vmin = 0;
        float vmax = 100;

        float dv;
        float r = 1.0f, g = 1.0f, b = 1.0f;
        float a = 0.5f;

        if (new_message->color.a == 0.0f and new_message->color.r == 0.0f and new_message->color.g == 0.0f and new_message->color.b == 0.0f) {

            if (v < vmin)
                v = vmin;
            if (v > vmax)
                v = vmax;
            dv = vmax - vmin;
            if (v < (vmin + 0.25f * dv)) {
                r = 0.0f;
                g = 4.0f * (v - vmin) / dv;
            } else if (v < (vmin + 0.5f * dv)) {
                r = 0.0f;
                b = 1.0f + 4.0f * (vmin + 0.25f * dv - v) / dv;
            } else if (v < (vmin + 0.75f * dv)) {
                r = 4.0f * (v - vmin - 0.5f * dv) / dv;
                b = 0.0f;
            } else {
                g = 1.0f + 4.0f * (vmin + 0.75f * dv - v) / dv;
                b = 0.0f;
            }



//            ROS_INFO_STREAM("sin: ");
//            ROS_INFO_STREAM("r: " << r);
//            ROS_INFO_STREAM("g: " << g);
//            ROS_INFO_STREAM("b: " << b);



        } else {
            r = new_message->color.r;
            g = new_message->color.g;
            b = new_message->color.b;
            a = new_message->color.a;
//            ROS_INFO_STREAM("con: ");
//            ROS_INFO_STREAM("r: " << r);
//            ROS_INFO_STREAM("g: " << g);
//            ROS_INFO_STREAM("b: " << b);
        }

        //        if (new_message->oncoming) {
        //            r, g, b = 0;
        //        }
        //
        //        if (new_message->bridge_object) {
        //            r, g, b = 1;
        //        }


        shape_->setColor(r, g, b, a);
    }

    void ShapeObstacle::update(float deltaTime) {
        //        if (animationState_) {
        //            animationState_->addTime(deltaTime * animationSpeed_);
        //        }
    }

    S_MaterialPtr ShapeObstacle::getMaterials() {
        S_MaterialPtr materials;
        extractMaterials(shape_->getEntity(), materials);
        return materials;
    }

}
