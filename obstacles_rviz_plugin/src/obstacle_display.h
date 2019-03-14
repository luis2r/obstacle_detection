/*
 * Copyright (c) 2008, Willow Garage, Inc.
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

#ifndef OBSTACLES_RVIZ_PLUGIN_MARKER_DISPLAY_H
#define OBSTACLES_RVIZ_PLUGIN_MARKER_DISPLAY_H

#include <map>
#include <set>

#include <boost/thread/mutex.hpp>
#include <boost/shared_ptr.hpp>

#ifndef Q_MOC_RUN
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#endif

//#include <animated_obstacle_msgs/AnimatedObstacle.h>
//#include <animated_obstacle_msgs/AnimatedObstacleArray.h>

#include <msgs_perception/ObstacleArray.h>
#include <msgs_perception/Obstacle.h>

#include "rviz/display.h"
#include "rviz/properties/bool_property.h"
#include "rviz/selection/forwards.h"

using namespace rviz;

namespace rviz {
    class IntProperty;
    class Object;
    class RosTopicProperty;
}

namespace obstacles_rviz_plugin {
    class ObstacleBase;
    class ObstacleNamespace;
    class ObstacleSelectionHandler;

    typedef boost::shared_ptr<ObstacleSelectionHandler> ObstacleSelectionHandlerPtr;
    typedef boost::shared_ptr<ObstacleBase> ObstacleBasePtr;
    typedef std::pair<std::string, int32_t> ObstacleID;

    /**
     * \class ObstacleDisplay
     * \brief Displays "obstacles" sent in by other ROS nodes on the "visualization_obstacle" topic
     *
     * Obstacles come in as msgs_perception::Obstacle messages.  See the AnimatedObstacle message for more information.
     */
    class ObstacleDisplay : public rviz::Display {
        Q_OBJECT
    public:
        ObstacleDisplay();
        virtual ~ObstacleDisplay();

        virtual void onInitialize();

        virtual void update(float wall_dt, float ros_dt);

        virtual void fixedFrameChanged();
        virtual void reset();

        void deleteObstacle(ObstacleID id);

        void setObstacleStatus(ObstacleID id, StatusLevel level, const std::string& text);
        void deleteObstacleStatus(ObstacleID id);

    protected:
        virtual void onEnable();
        virtual void onDisable();

        /** @brief Subscribes to the "visualization_obstacle" and
         * "visualization_obstacle_array" topics. */
        virtual void subscribe();

        /** @brief Unsubscribes from the "visualization_obstacle"
         * "visualization_obstacle_array" topics. */
        virtual void unsubscribe();

        /** @brief Process a ObstacleArray message. */
        void incomingObstacleArray(const msgs_perception::ObstacleArray::ConstPtr& array);

        ros::Subscriber array_sub_;

        RosTopicProperty* obstacle_topic_property_;
        IntProperty* queue_size_property_;

        private 
Q_SLOTS:
        void updateQueueSize();
        void updateTopic();

    private:
        /** @brief Delete all the obstacles within the given namespace. */
        void deleteObstaclesInNamespace(const std::string& ns);

        /**
         * \brief Removes all the obstacles
         */
        void clearObstacles();

        /**
         * \brief Processes a obstacle message
         * @param message The message to process
         */
        void processMessage(const msgs_perception::Obstacle::ConstPtr& message);
        /**
         * \brief Processes an "Add" obstacle message
         * @param message The message to process
         */
        void processAdd(const msgs_perception::Obstacle::ConstPtr& message);
        /**
         * \brief Processes a "Delete" obstacle message
         * @param message The message to process
         */
        void processDelete(const msgs_perception::Obstacle::ConstPtr& message);

        /**
         * \brief ROS callback notifying us of a new obstacle
         */
        void incomingObstacle(const msgs_perception::Obstacle::ConstPtr& obstacle);

        void failedObstacle(const ros::MessageEvent<msgs_perception::Obstacle>& obstacle_evt, tf::FilterFailureReason reason);

        typedef std::map<ObstacleID, ObstacleBasePtr> M_IDToObstacle;
        typedef std::set<ObstacleBasePtr> S_ObstacleBase;
        M_IDToObstacle obstacles_; ///< Map of obstacle id to the obstacle info structure
        S_ObstacleBase obstacles_with_expiration_;
        S_ObstacleBase frame_locked_obstacles_;
        typedef std::vector<msgs_perception::Obstacle::ConstPtr> V_ObstacleMessage;
        V_ObstacleMessage message_queue_; ///< Obstacle message queue.  Messages are added to this as they are received, and then processed
        ///< in our update() function
        boost::mutex queue_mutex_;

        message_filters::Subscriber<msgs_perception::Obstacle> sub_;
        tf::MessageFilter<msgs_perception::Obstacle>* tf_filter_;

        typedef QHash<QString, obstacles_rviz_plugin::ObstacleNamespace*> M_Namespace;
        M_Namespace namespaces_;

        Property* namespaces_category_;

        friend class ObstacleNamespace;
    };

    /** @brief Manager of a single obstacle namespace.  Keeps a hash from
     * obstacle IDs to ObstacleBasePtr, and creates or destroys them when . */
    class ObstacleNamespace : public BoolProperty {
        Q_OBJECT
    public:
        ObstacleNamespace(const QString& name, Property* parent_property, ObstacleDisplay* owner);

        bool isEnabled() const {
            return getBool();
        }

        public 
Q_SLOTS:
        void onEnableChanged();

    private:
        ObstacleDisplay* owner_;
    };

} // namespace obstacles_rviz_plugin

#endif /* RVIZ_MARKER_DISPLAY_H */
