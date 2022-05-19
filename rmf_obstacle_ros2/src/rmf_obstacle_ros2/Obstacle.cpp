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

#include <rmf_obstacle_ros2/Obstacle.hpp>

#include <octomap/octomap.h>

#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace rmf_obstacle_ros2 {

//==============================================================================
ObstacleData convert(const PointCloud& msg)
{
  ObstacleData data;
  octomap::OcTree tree(data.RESOLUTION);

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
  // We assume the point cloud data is in global RMF coordinates.
  const octomap::point3d sensor_origin(0.0, 0.0, 0.0);
  tree.insertPointCloud(cloud, sensor_origin);

  // Write binary data to ObstacleData msg
  std::stringstream datastream;
  tree.writeBinaryData(datastream);
  const std::string datastring = datastream.str();
  data.data = std::vector<int8_t>(datastring.begin(), datastring.end());

  return data;
}

//==============================================================================
PointCloud convert(
  const Header& header,
  const ObstacleData& msg)
{
  PointCloud cloud;
  cloud.header = header;

  octomap::OcTree tree(msg.RESOLUTION);
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

  // Note that the non-trivial call to tree->end_leafs() should be done only once for efficiency
  for(auto leaf_it = tree.begin_leafs(), end = tree.end_leafs(); leaf_it != end; ++leaf_it)
  {
    const auto& p = leaf_it.getCoordinate();
    *iter_x = p.x();
    *iter_y = p.y();
    *iter_z = p.z();
    ++iter_x;
    ++iter_y;
    ++iter_z;
  }

  // TODO(YV): Fill other fields
  return cloud;
}

} // namespace  rmf_obstacle_ros2
