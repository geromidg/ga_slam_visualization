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
    ros::Publisher rawMapPublisher = nodeHandle.
            advertise<grid_map_msgs::GridMap>("raw_map", 1);
    ros::Rate loop_rate(0.5);

    while (ros::ok()) {
        grid_map::GridMap rawMap;
        grid_map_msgs::GridMap message;

        loadGridMap(rawMap, "/home/dimi/grid_map.cereal");

        grid_map::GridMapRosConverter::toMessage(rawMap, message);
        message.info.header.frame_id = "map";

        rawMapPublisher.publish(message);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

