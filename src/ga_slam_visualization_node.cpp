#include <fstream>

#include <Eigen/Core>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <eigen_conversions/eigen_msg.h>

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

    const std::string frameId = "map";

    Eigen::Affine3d pose;
    geometry_msgs::Pose poseMessage;
    geometry_msgs::PoseStamped poseStampedMessage;

    grid_map::GridMap rawMap;
    grid_map_msgs::GridMap rawMapMessage;
    grid_map::Time lastUpdateTimestamp, currentUpdateTimestamp;

    lastUpdateTimestamp = 0;

    while (ros::ok()) {
        loadPose(pose, "/tmp/ga_slam_pose.cereal");
        loadGridMap(rawMap, "/tmp/ga_slam_map.cereal");

        rawMap.setFrameId(frameId);
        currentUpdateTimestamp = rawMap.getTimestamp();

        if (currentUpdateTimestamp != lastUpdateTimestamp) {
            lastUpdateTimestamp = currentUpdateTimestamp;

            grid_map::GridMapRosConverter::toMessage(rawMap, rawMapMessage);
            rawMapPublisher.publish(rawMapMessage);

            tf::poseEigenToMsg(pose, poseMessage);
            poseStampedMessage.pose = poseMessage;
            poseStampedMessage.header.stamp.fromNSec(currentUpdateTimestamp);
            poseStampedMessage.header.frame_id = frameId;
            posePublisher.publish(poseStampedMessage);
        }

        rate.sleep();
    }

    return 0;
}

