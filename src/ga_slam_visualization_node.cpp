#include <fstream>

#include <Eigen/Core>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>

#include "grid_map_cereal/GridMapCereal.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "ga_slam_visualization");
    ros::NodeHandle nodeHandle("~");
    ros::Rate rate(1);
    ros::Publisher rawMapPublisher = nodeHandle.
            advertise<grid_map_msgs::GridMap>("raw_map", 1, true);
    ros::Publisher globalMapPublisher = nodeHandle.
            advertise<grid_map_msgs::GridMap>("global_map", 1, true);
    ros::Publisher posePublisher = nodeHandle.
            advertise<geometry_msgs::PoseStamped>("pose", 1, true);
    ros::Publisher pathPublisher = nodeHandle.
            advertise<nav_msgs::Path>("path", 1, true);
    tf::TransformBroadcaster tfBroadcaster;

    const std::string mapFrameId = "map";
    const std::string bodyFrameId = "body_base";

    Eigen::Affine3d pose;
    geometry_msgs::Pose poseMessage;
    geometry_msgs::PoseStamped poseStampedMessage;
    nav_msgs::Path pathMessage;
    tf::Transform bodyToMapTF;

    grid_map::GridMap rawMap, globalMap;
    grid_map_msgs::GridMap mapMessage;
    grid_map::Time lastRawMapStamp, currentRawMapStamp;
    grid_map::Time lastGlobalMapStamp, currentGlobalMapStamp;

    lastRawMapStamp = 0;
    lastGlobalMapStamp = 0;

    while (ros::ok()) {
        loadPose(pose, "/tmp/ga_slam_pose.cereal");

        loadGridMap(rawMap, "/tmp/ga_slam_local_map.cereal");
        rawMap.setFrameId(mapFrameId);
        currentRawMapStamp = rawMap.getTimestamp();

        if (currentRawMapStamp != lastRawMapStamp) {
            grid_map::GridMapRosConverter::toMessage(rawMap, mapMessage);
            rawMapPublisher.publish(mapMessage);

            tf::poseEigenToMsg(pose, poseMessage);
            poseStampedMessage.pose = poseMessage;
            poseStampedMessage.header.stamp.fromNSec(currentRawMapStamp);
            poseStampedMessage.header.frame_id = mapFrameId;
            posePublisher.publish(poseStampedMessage);

            tf::poseEigenToTF(pose, bodyToMapTF);
            tfBroadcaster.sendTransform(tf::StampedTransform(
                    bodyToMapTF, ros::Time::now(), mapFrameId, bodyFrameId));

            if (currentRawMapStamp < lastRawMapStamp)
                pathMessage.poses.clear();
            pathMessage.header.stamp.fromNSec(currentRawMapStamp);
            pathMessage.header.frame_id = mapFrameId;
            pathMessage.poses.push_back(poseStampedMessage);
            pathPublisher.publish(pathMessage);

            lastRawMapStamp = currentRawMapStamp;
        }

        loadGridMap(globalMap, "/tmp/ga_slam_global_map.cereal");
        globalMap.setFrameId(mapFrameId);
        currentGlobalMapStamp = globalMap.getTimestamp();

        if (currentGlobalMapStamp != lastGlobalMapStamp) {
            grid_map::GridMapRosConverter::toMessage(globalMap, mapMessage);
            globalMapPublisher.publish(mapMessage);

            lastGlobalMapStamp = currentGlobalMapStamp;
        }

        rate.sleep();
    }

    return 0;
}

