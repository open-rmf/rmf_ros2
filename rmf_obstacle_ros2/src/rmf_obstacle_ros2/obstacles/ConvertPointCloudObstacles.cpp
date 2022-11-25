/*
 * Copyright (C) 2022 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <rmf_obstacle_ros2/obstacles/Convert.hpp>

#include <octomap/OcTree.h>

#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace rmf_obstacle_ros2 {

//==============================================================================
template<>
void fill_obstacle_data(const PointCloud& msg, Obstacle& obstacle)
{
  const double resolution = obstacle.data_resolution > 0 ?
    obstacle.data_resolution : 0.1;
  octomap::OcTree tree(resolution);

  // First convert to octomap::Pointcloud
  octomap::Pointcloud cloud;
  cloud.reserve(msg.data.size() / msg.point_step);

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(msg, "z");

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
  {
    // Check if the point is invalid
    if (!std::isnan(*iter_x) && !std::isnan(*iter_y) && !std::isnan(*iter_z))
    {
      cloud.push_back(*iter_x, *iter_y, *iter_z);
    }
  }
  // We assume the point cloud data has its origin at the frame id specified
  const octomap::point3d sensor_origin(0.0, 0.0, 0.0);
  tree.insertPointCloud(cloud, sensor_origin);

  // Write binary data to ObstacleData msg
  std::stringstream datastream;
  tree.writeBinaryData(datastream);
  const std::string datastring = datastream.str();
  obstacle.data = std::vector<int8_t>(datastring.begin(), datastring.end());
}

//==============================================================================
template<>
PointCloud convert(
  const Obstacle& msg)
{
  PointCloud cloud;
  cloud.header = msg.header;

  octomap::OcTree tree(msg.data_resolution);
  // Construct octree
  if (msg.data.empty())
    return cloud;
  std::stringstream datastream;
  datastream.write((const char*) &msg.data[0], msg.data.size());
  tree.readBinaryData(datastream);

  sensor_msgs::PointCloud2Modifier pcd_modifier(cloud);
  pcd_modifier.resize(tree.calcNumNodes());

  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

  int number_of_points = 0;
  // Note that the non-trivial call to tree->end_leafs() should be done only once for efficiency
  for (auto leaf_it = tree.begin_leafs(),
    end = tree.end_leafs(); leaf_it != end;
    ++leaf_it)
  {
    const auto& p = leaf_it.getCoordinate();
    *iter_x = p.x();
    *iter_y = p.y();
    *iter_z = p.z();
    ++iter_x;
    ++iter_y;
    ++iter_z;
    ++number_of_points;
  }

  cloud.is_bigendian = false;
  // If the cloud is unordered, height is 1 and width is the length of
  // the point cloud.
  cloud.height = 1;
  cloud.width = number_of_points;
  // Length of a point in bytes, each point has 3 float coordinates
  cloud.point_step = sizeof(float) * 3;
  cloud.row_step = cloud.width * cloud.point_step;

  sensor_msgs::msg::PointField point_field;
  point_field.name = "x";
  point_field.offset = 0;
  point_field.datatype = sensor_msgs::msg::PointField::FLOAT32;
  point_field.count = 1;

  cloud.fields.push_back(point_field);
  point_field.name = "y";
  point_field.offset = 4;
  point_field.datatype = sensor_msgs::msg::PointField::FLOAT32;
  point_field.count = 1;

  cloud.fields.push_back(point_field);

  point_field.name = "z";
  point_field.offset = 8;
  point_field.datatype = sensor_msgs::msg::PointField::FLOAT32;
  point_field.count = 1;

  cloud.fields.push_back(point_field);

  cloud.is_dense = true;
  return cloud;
}

} // namespace  rmf_obstacle_ros2
