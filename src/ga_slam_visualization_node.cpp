#include <fstream>

#include <Eigen/Core>

#include <ros/ros.h>

#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>

#include <cereal/types/string.hpp>
#include <cereal/types/vector.hpp>
#include <cereal/types/unordered_map.hpp>
#include <cereal/archives/binary.hpp>

namespace cereal {

template <class Archive, class _Scalar, int _Rows, int _Cols,
        int _Options, int _MaxRows, int _MaxCols> inline
void save(Archive& archive, const Eigen::Matrix<_Scalar, _Rows, _Cols,
        _Options, _MaxRows, _MaxCols>& m) {
    int rows = m.rows();
    int cols = m.cols();

    archive(rows);
    archive(cols);
    archive(binary_data(m.data(), rows * cols * sizeof(_Scalar)));
}

template <class Archive, class _Scalar, int _Rows, int _Cols,
        int _Options, int _MaxRows, int _MaxCols> inline
void load(Archive& archive, Eigen::Matrix<_Scalar, _Rows, _Cols,
        _Options, _MaxRows, _MaxCols> & m) {
    int rows;
    int cols;

    archive(rows);
    archive(cols);

    m.resize(rows, cols);
    archive(binary_data(m.data(),
            static_cast<std::size_t>(rows * cols * sizeof(_Scalar))));
}

template <class Archive>
void save(Archive& archive, const grid_map::GridMap& map) {
    std::string frameId = map.getFrameId();
    grid_map::Time timestamp = map.getTimestamp();
    std::vector<std::string> layers = map.getLayers();
    std::vector<std::string> basicLayers = map.getBasicLayers();
    grid_map::Length length = map.getLength();
    double resolution = map.getResolution();
    grid_map::Position position = map.getPosition();
    grid_map::Size size = map.getSize();
    grid_map::Index startIndex = map.getStartIndex();

    std::unordered_map<std::string, grid_map::Matrix> data;
    for (const auto& layer : layers)
        data[layer] = map.get(layer);

    archive(frameId);
    archive(timestamp);
    archive(data);
    archive(layers);
    archive(basicLayers);
    archive(resolution);

    // archive(length);
    // archive(position);
    // archive(startIndex);
}

template <class Archive>
void load(Archive& archive, grid_map::GridMap& map) {
    std::string frameId;
    grid_map::Time timestamp;
    std::unordered_map<std::string, grid_map::Matrix> data;
    std::vector<std::string> layers;
    std::vector<std::string> basicLayers;
    grid_map::Length length;
    double resolution;
    grid_map::Position position;
    grid_map::Size size;
    grid_map::Index startIndex;

    archive(frameId);
    archive(timestamp);
    archive(data);
    archive(layers);
    archive(basicLayers);
    archive(resolution);

    // archive(length);
    // archive(position);
    // archive(startIndex);

    map.setFrameId(frameId);
    map.setTimestamp(timestamp);

    // map.setGeometry(length, resolution, position);
    map.setGeometry(grid_map::Length(10., 10.), resolution,
            grid_map::Position(0., 0.));

    for (const auto& layerData : data)
        map.add(layerData.first, layerData.second);

    map.setBasicLayers(basicLayers);

    // map.setStartIndex(startIndex);
}

}  // namespace cereal

int main(int argc, char **argv) {
    ros::init(argc, argv, "ga_slam_visualization");
    ros::NodeHandle nodeHandle("~");
    ros::Publisher rawMapPublisher = nodeHandle.
            advertise<grid_map_msgs::GridMap>("raw_map", 1);
    ros::Rate loop_rate(0.5);

    while (ros::ok()) {
        grid_map::GridMap rawMap;
        grid_map_msgs::GridMap message;

        {
            std::ifstream streamIn("/home/dimi/grid_map.cereal",
                    std::ios::binary);
            cereal::BinaryInputArchive archiveIn(streamIn);
            archiveIn(rawMap);
        }

        grid_map::GridMapRosConverter::toMessage(rawMap, message);
        message.info.header.frame_id = "map";

        rawMapPublisher.publish(message);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

