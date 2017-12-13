#include <fstream>

#include <Eigen/Core>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
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
    ros::Publisher posePublisher = nodeHandle.
            advertise<geometry_msgs::PoseStamped>("pose", 1, true);
    tf::TransformBroadcaster tfBroadcaster;
    tf::Transform bodyToMapTF;

    const std::string mapFrameId = "map";
    const std::string bodyFrameId = "body_base";

    Eigen::Affine3d pose;
    geometry_msgs::Pose poseMessage;
    geometry_msgs::PoseStamped poseStampedMessage;

    grid_map::GridMap rawMap;
    grid_map_msgs::GridMap rawMapMessage;
    grid_map::Time lastStamp, currentStamp;

    lastStamp = 0;

    while (ros::ok()) {
        loadPose(pose, "/tmp/ga_slam_pose.cereal");
        loadGridMap(rawMap, "/tmp/ga_slam_map.cereal");

        rawMap.setFrameId(mapFrameId);
        currentStamp = rawMap.getTimestamp();

        if (currentStamp != lastStamp) {
            lastStamp = currentStamp;

            grid_map::GridMapRosConverter::toMessage(rawMap, rawMapMessage);
            rawMapPublisher.publish(rawMapMessage);

            tf::poseEigenToMsg(pose, poseMessage);
            poseStampedMessage.pose = poseMessage;
            poseStampedMessage.header.stamp.fromNSec(currentStamp);
            poseStampedMessage.header.frame_id = mapFrameId;
            posePublisher.publish(poseStampedMessage);

            tf::poseEigenToTF(pose, bodyToMapTF);
            tfBroadcaster.sendTransform(tf::StampedTransform(
                    bodyToMapTF, ros::Time::now(), bodyFrameId, mapFrameId));
        }

        rate.sleep();
    }

    return 0;
}

