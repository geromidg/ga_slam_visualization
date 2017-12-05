#include <fstream>

#include <Eigen/Core>

#include <ros/ros.h>

#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>

#include "grid_map_cereal/GridMapCereal.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "ga_slam_visualization");
    ros::NodeHandle nodeHandle("~");
    ros::Rate rate(0.5);
    ros::Publisher rawMapPublisher = nodeHandle.
            advertise<grid_map_msgs::GridMap>("raw_map", 1, true);

    grid_map::GridMap rawMap;
    grid_map_msgs::GridMap message;

    while (ros::ok()) {
        loadGridMap(rawMap, "/tmp/ga_slam_map.cereal");
        rawMap.setFrameId("map");

        grid_map::GridMapRosConverter::toMessage(rawMap, message);
        rawMapPublisher.publish(message);

        rate.sleep();
    }

    return 0;
}

