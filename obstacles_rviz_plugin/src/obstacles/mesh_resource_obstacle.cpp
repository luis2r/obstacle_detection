/*
 * Copyright (c) 2010, Willow Garage, Inc.
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

#include "mesh_resource_obstacle.h"

#include "obstacle_selection_handler.h"
#include "../obstacle_display.h"
#include "rviz/selection/selection_manager.h"

#include "rviz/display_context.h"
#include "rviz/mesh_loader.h"

#include <resource_retriever/retriever.h>
#include <boost/filesystem.hpp>

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreEntity.h>
#include <OgreSubEntity.h>
#include <OgreMaterialManager.h>
#include <OgreTextureManager.h>
#include <OgreSharedPtr.h>
#include <OgreTechnique.h>
#include <OgreAnimation.h>

#include <math.h>

namespace fs = boost::filesystem;

namespace obstacles_rviz_plugin {

    MeshResourceObstacle::MeshResourceObstacle(ObstacleDisplay* owner, DisplayContext* context, Ogre::SceneNode* parent_node)
    : ObstacleBase(owner, context, parent_node)
    , entity_(0), animationState_(0), animationSpeed_(1.0f) {
    }

    MeshResourceObstacle::~MeshResourceObstacle() {
        reset();
    }

    void MeshResourceObstacle::reset() {
        //destroy entity
        if (entity_) {
            context_->getSceneManager()->destroyEntity(entity_);
            entity_ = 0;
        }

        // destroy all the materials we've created
        S_MaterialPtr::iterator it;
        for (it = materials_.begin(); it != materials_.end(); it++) {
            Ogre::MaterialPtr material = *it;
            if (!material.isNull()) {
                material->unload();
                Ogre::MaterialManager::getSingleton().remove(material->getName());
            }
        }
        materials_.clear();
    }

    class RosPackagePathResourceLoadingListener : public Ogre::ResourceLoadingListener {
    public:

        RosPackagePathResourceLoadingListener(const fs::path& parentPath) : _parentPath(parentPath) {
        }

        /** This event is called when a resource beings loading. */
        virtual Ogre::DataStreamPtr resourceLoading(const Ogre::String &name, const Ogre::String &group, Ogre::Resource *resource) {
            fs::path absolutePath = _parentPath / name;
            ROS_INFO_STREAM("RosPackagePathResourceLoadingListener loading resource: " << absolutePath.string());

            try {
                resource_retriever::Retriever retriever;
                _lastResource = retriever.get(absolutePath.string()); // not thread-safe!
                return Ogre::DataStreamPtr(new Ogre::MemoryDataStream(_lastResource.data.get(), _lastResource.size));
            } catch (resource_retriever::Exception& e) {
                ROS_ERROR("In RosPackagePathResourceLoadingListener: %s", e.what());
                return Ogre::DataStreamPtr();
            }
        }

        virtual void resourceStreamOpened(const Ogre::String &name, const Ogre::String &group, Ogre::Resource *resource, Ogre::DataStreamPtr& dataStream) {
        }

        virtual bool resourceCollision(Ogre::Resource *resource, Ogre::ResourceManager *resourceManager) {
            return false;
        }

    private:
        const fs::path& _parentPath;
        resource_retriever::MemoryResource _lastResource;
    };

    void MeshResourceObstacle::onNewMessage(const ObstacleConstPtr& old_message, const ObstacleConstPtr& new_message) {
        //ROS_ASSERT(new_message->type == msgs_perception::Obstacle::MESH_RESOURCE);

        bool need_color = false;

        scene_node_->setVisible(false);

        if (!entity_ /*
        ||
                old_message->mesh_resource != new_message->mesh_resource ||
                old_message->mesh_use_embedded_materials != new_message->mesh_use_embedded_materials
                */)
                    if (!entity_ ||
                            old_message->type != new_message->type /*||
                            old_message->mesh_use_embedded_materials != new_message->mesh_use_embedded_materials*/) 
        {
            reset();

            //            if (new_message->mesh_resource.empty()) {
            //                return;
            //            }


            //            fs::path model_path(new_message->mesh_resource);
            //                                    fs::path model_path('package://animated_marker_tutorial/meshes/animated_walking_man.mesh');
            //            fs::path parent_path(model_path.parent_path());
            //            
            fs::path model_path;
            fs::path parent_path;
            ////            const std::string& new_message_mesh_resource;
            //            if (new_message->type2 == 0) {
            model_path = "package://obstacles_rviz_plugin/meshes/animated_walking_man.mesh";
            //                            model_path = "package://animated_marker_tutorial/meshes/animated_walking_man.mesh";
            parent_path = "package://obstacles_rviz_plugin/meshes";
            //                            parent_path = "package://animated_marker_tutorial/meshes";
            const std::string& new_message_mesh_resource = "package://obstacles_rviz_plugin/meshes/animated_walking_man.mesh";
            //                            const std::string& new_message_mesh_resource = "package://animated_marker_tutorial/meshes/animated_walking_man.mesh";
            //                ROS_WARN("case 1");
            //            } else {
            //                if (new_message->type2 == 1) {
            //                    model_path = "package://animated_marker_tutorial/meshes/robot.mesh";
            //                    parent_path = "package://animated_marker_tutorial/meshes";
            ////                    const std::string& new_message_mesh_resource = "package://animated_marker_tutorial/meshes/fish.mesh";
            //                    ROS_WARN("case 2");
            //                } else {
            //                    if (new_message->type2 == -1) {
            //                        model_path = "package://animated_marker_tutorial/meshes/robot.mesh";
            //                        parent_path = "package://animated_marker_tutorial/meshes";
            ////                        const std::string& new_message_mesh_resource = "package://animated_marker_tutorial/meshes/robot.mesh";
            //                        ROS_WARN("case 3");
            //                    } else {
            //                        model_path = "package://animated_marker_tutorial/meshes/jaiqua.mesh";
            //                        parent_path = "package://animated_marker_tutorial/meshes";
            ////                        const std::string& new_message_mesh_resource = "package://animated_marker_tutorial/meshes/jaiqua.mesh";
            //                        ROS_WARN("case 4");
            //
            //                    }
            //                }
            //            }







            //              std::stringstream ss1;
            //            ss1 << "path model_path  " << model_path << "";
            //            ROS_WARN("%s", ss1.str().c_str());
            //
            //
            //            std::stringstream ss2;
            //            ss2 << "parent_path  " << parent_path << "";
            //            ROS_WARN("%s", ss2.str().c_str());
            //            
            //            std::stringstream ss3;
            //            ss3 << "new_message->mesh_resourc


            Ogre::ResourceLoadingListener *newListener = new RosPackagePathResourceLoadingListener(parent_path), *oldListener = Ogre::ResourceGroupManager::getSingleton().getLoadingListener();
            Ogre::ResourceGroupManager::getSingleton().setLoadingListener(newListener);

            bool loadFailed = loadMeshFromResource(new_message_mesh_resource).isNull();

            //            bool loadFailed = loadmeshFromResource("package://animated_marker_tutorial/pedestrianmeshes/animated_walking_man.pedestrianmesh").isNull();

            //            bool loadFailed;
            //            if (new_message->type2 == 0) {
            //                loadFailed = loadmeshFromResource("package://animated_marker_tutorial/pedestrianmeshes/animated_walking_man.pedestrianmesh").isNull();
            //                ROS_WARN("case 1");
            //                
            //            } else {
            //                if (new_message->type2 == 1) {
            //                    loadFailed = loadmeshFromResource("package://animated_marker_tutorial/pedestrianmeshes/robot.pedestrianmesh").isNull();
            //                    ROS_WARN("case 2");
            //                    
            //                } else {
            //                    if (new_message->type2 == -1) {
            //                        loadFailed = loadmeshFromResource("package://animated_marker_tutorial/pedestrianmeshes/robot.pedestrianmesh").isNull();
            //                        ROS_WARN("case 3");
            //                        
            //                    } else {
            //                        loadFailed = loadmeshFromResource("package://animated_marker_tutorial/pedestrianmeshes/jaiqua.pedestrianmesh").isNull();
            //                        ROS_WARN("case 4");
            //                        
            //
            //                    }
            //                }
            //            }



            //            bool loadFailed = loadMeshFromResource(new_message->mesh_resource).isNull();
            Ogre::ResourceGroupManager::getSingleton().setLoadingListener(oldListener);

            delete newListener;


            if (loadFailed) {
                std::stringstream ss;
                ss << "mesh resource obstacle [" << getStringID() << "] could not load [" << new_message_mesh_resource << "]";
                //                ss << "mesh resource obstacle [" << getStringID() << "] could not load [" << new_message->mesh_resource << "]";
                if (owner_) {
                    owner_->setObstacleStatus(getID(), StatusProperty::Error, ss.str());
                }
                ROS_WARN("%s", ss.str().c_str());
                return;
            }

            static uint32_t count = 0;
            std::stringstream ss;
            ss << "mesh_resource_obstacle_" << count++;
            std::string id = ss.str();
            //            entity_ = context_->getSceneManager()->createEntity(id, new_message->mesh_resource);
            entity_ = context_->getSceneManager()->createEntity(id, new_message_mesh_resource);
            scene_node_->attachObject(entity_);
            need_color = true;

            // set up animation
            std::string nameOfAnimationState = "";
            Ogre::AnimationStateSet *animationStates = entity_->getAllAnimationStates();
            if (animationStates != NULL) {
                Ogre::AnimationStateIterator animationsIterator = animationStates->getAnimationStateIterator();
                while (animationsIterator.hasMoreElements()) {
                    Ogre::AnimationState *animationState = animationsIterator.getNext();
                    if (animationState->getAnimationName() == nameOfAnimationState || nameOfAnimationState.empty()) {
                        animationState_ = animationState;
                        ROS_DEBUG_STREAM("Enabling animation state " << animationState->getAnimationName());
                        animationState->setLoop(true);
                        animationState->setEnabled(true);
                        //animationState->addTime((rand() * 100 / 100.0));
                    }
                }
            }

            // create a default material for any sub-entities which don't have their own.
            ss << "Material";
            Ogre::MaterialPtr default_material = Ogre::MaterialManager::getSingleton().create(ss.str(), "rviz");
            default_material->setReceiveShadows(false);
            default_material->getTechnique(0)->setLightingEnabled(true);
            default_material->getTechnique(0)->setAmbient(0.5, 0.5, 0.5);
            materials_.insert(default_material);

            //            if (new_message->mesh_use_embedded_materials) {
            //                // make clones of all embedded materials so selection works correctly
            //                S_MaterialPtr materials = getMaterials();
            //
            //                S_MaterialPtr::iterator it;
            //                for (it = materials.begin(); it != materials.end(); it++) {
            //                    if ((*it)->getName() != "BaseWhiteNoLighting") {
            //                        Ogre::MaterialPtr new_material = (*it)->clone(id + (*it)->getName());
            //                        materials_.insert(new_material);
            //                    }
            //                }
            //
            //                // make sub-entities use cloned materials
            //                for (uint32_t i = 0; i < entity_->getNumSubEntities(); ++i) {
            //                    std::string mat_name = entity_->getSubEntity(i)->getMaterialName();
            //                    if (mat_name != "BaseWhiteNoLighting") {
            //                        entity_->getSubEntity(i)->setMaterialName(id + mat_name);
            //                    } else {
            //                        // BaseWhiteNoLighting is the default material Ogre uses
            //                        // when it sees a mesh with no material.  Here we replace
            //                        // that with our default_material which gets colored with
            //                        // new_message->color.
            //                        entity_->getSubEntity(i)->setMaterial(default_material);
            //                    }
            //                }
            //            } else {
            entity_->setMaterial(default_material);
            //            }

            handler_.reset(new ObstacleSelectionHandler(this, ObstacleID(new_message->ns, new_message->id), context_));
            handler_->addTrackedObject(entity_);
        }

        //        if (need_color ||
        //                old_message->color.r != new_message->color.r ||
        //                old_message->color.g != new_message->color.g ||
        //                old_message->color.b != new_message->color.b ||
        //                old_message->color.a != new_message->color.a) {
        //            float r = new_message->color.r;
        //            float g = new_message->color.g;
        //            float b = new_message->color.b;
        //            float a = new_message->color.a;

        //            float r = 0.1;
        //            float g = 1;
        //            float b = 0.1;
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





        // Old way was to ignore the color and alpha when using embedded
        // materials, which meant you could leave them unset, which means
        // 0.  Since we now USE the color and alpha values, leaving them
        // all 0 will mean the object will be invisible.  Therefore detect
        // the situation where RGBA are all 0 and treat that the same as
        // all 1 (full white).
        //        if (new_message->mesh_use_embedded_materials && r == 0 && g == 0 && b == 0 && a == 0) {
        //            r = 1;
        //            g = 1;
        //            b = 1;
        //            a = 1;
        //        }

        Ogre::SceneBlendType blending;
        bool depth_write;

        if (a < 0.9998) {
            blending = Ogre::SBT_TRANSPARENT_ALPHA;
            depth_write = false;
        } else {
            blending = Ogre::SBT_REPLACE;
            depth_write = true;
        }

        S_MaterialPtr::iterator it;
        for (it = materials_.begin(); it != materials_.end(); it++) {
            Ogre::Technique* technique = (*it)->getTechnique(0);

            //                technique->setAmbient(r * 0.5, g * 0.5, b * 0.5);
            //                technique->setDiffuse(r, g, b, a);

            technique->setAmbient(r * 0.5, g * 0.5, b * 0.5);
            technique->setDiffuse(r, g, b, a);

            technique->setSceneBlending(blending);
            technique->setDepthWriteEnabled(depth_write);
            technique->setLightingEnabled(true);
        }
        //        }

        animationSpeed_ = new_message->animation_speed;

        Ogre::Vector3 pos, scale;
        Ogre::Quaternion orient;
        transform(new_message, pos, orient, scale);

        scene_node_->setVisible(true);
        setPosition(pos);
        setOrientation(orient);

        scene_node_->setScale(scale);
    }

    void MeshResourceObstacle::update(float deltaTime) {
        if (animationState_) {
            animationState_->addTime(deltaTime * animationSpeed_);
        }
    }

    S_MaterialPtr MeshResourceObstacle::getMaterials() {
        S_MaterialPtr materials;
        if (entity_) {
            extractMaterials(entity_, materials);
        }
        return materials;
    }

}

