/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include <iostream>
#include <cstdio>
#include <filesystem>
#include <zlib.h>


#include <gdal/gdal.h>
#include <gdal/ogrsf_frmts.h>

#include <nlohmann/json.hpp>

#include <rmf_traffic_ros2/agv/Graph.hpp>

namespace rmf_traffic_ros2 {

// Usage map[level_idx][truncated_x][truncated_y] = id;
// Truncation is to 1e-3 meters
using CoordsIdxHashMap = std::unordered_map<std::size_t, std::unordered_map<
  double, std::unordered_map<double, std::size_t>>>;

// Class to wrap temporary files so they are deleted on destruction
class GeopackageTmpFile
{
private:
  FILE *fd;

public:
  std::string filename;

  GeopackageTmpFile(const std::vector<unsigned char>& data)
  {
    // TODO use return value for errors
    char tmpnam[] = "/tmp/geopkgXXXXXX";
    int res = mkstemp(tmpnam);
    filename = tmpnam;
    fd = fopen(filename.c_str(), "wb");
    fwrite(&data[0], sizeof(char), data.size(), fd);
  }

  ~GeopackageTmpFile()
  {
    // Delete the tmp file
    fclose(fd);
    remove(filename.c_str());
  }
};

//==============================================================================
rmf_traffic::agv::Graph convert(const rmf_site_map_msgs::msg::SiteMap& from,
  int graph_idx, double wp_tolerance)
{
  CoordsIdxHashMap idx_map;
  rmf_traffic::agv::Graph graph;
  // Sqlite3 needs to work on a physical file, write the package to a tmp file
  // TODO delete file once done
  GDALAllRegister();
  // Not supported
  if (from.encoding == from.MAP_DATA_GEOJSON)
  {
    printf("converting GeoJSON map\n");
  }
  else if (from.encoding == from.MAP_DATA_GEOJSON_GZ)
  {
    printf("converting compressed GeoJSON map\n");
  }
  else
    return graph;  // unexpected encoding

  return graph;

  GeopackageTmpFile gpkg_file(from.data);
  GDALDatasetUniquePtr poDS(GDALDataset::Open(gpkg_file.filename.c_str(), GDAL_OF_VECTOR));
  // Iterate over vertices
  auto vertices_layer = poDS->GetLayerByName("vertices");
  while (const auto& feature = vertices_layer->GetNextFeature())
  {
    int level_idx = 0;
    for (const auto& field : feature)
    {
      if (strcmp(field.GetName(), "level_idx") == 0)
        level_idx = field.GetAsInteger();
      else if (strcmp(field.GetName(), "parameters") == 0)
      {
        // TODO parse parameters here
      }
    }
    const auto& name_feat = (*feature)["name"];
    const std::string name(name_feat.GetAsString());
    const auto& point = feature->GetGeometryRef()->toPoint();
    // Flatten geometry to extract 
    const Eigen::Vector2d location{
      point->getX(), point->getY()};
    // TODO map name
    std::string map_name("test");
    auto& wp = graph.add_waypoint(map_name, location);
    if (name.size() > 0 && !graph.add_key(name, wp.index()))
    {
      throw std::runtime_error(
              "Duplicated waypoint name [" + name + "]");
    }
    // Round x and y to wp_tolerance
    double rounded_x = std::round(point->getX() / wp_tolerance) * wp_tolerance;
    double rounded_y = std::round(point->getY() / wp_tolerance) * wp_tolerance;
    idx_map[level_idx][rounded_x][rounded_y] = wp.index();
  }
  // Iterate over edges
  auto edges_layer = poDS->GetLayerByName("edges");
  while (const auto& feature = edges_layer->GetNextFeature())
  {
    int level_idx = 0;
    std::optional<double> speed_limit;
    std::optional<std::string> dock_name;
    bool is_bidirectional = false;
    bool is_correct_graph = false;
    for (const auto& field : feature)
    {
      if (strcmp(field.GetName(), "level_idx") == 0)
        level_idx = field.GetAsInteger();
      else if (strcmp(field.GetName(), "parameters") == 0)
      {
        const auto& params_str = field.GetAsString();
        nlohmann::json j = nlohmann::json::parse(params_str);
        auto graph_idx_it = j.find("graph_idx");
        if (graph_idx_it != j.end())
          is_correct_graph = (graph_idx_it->get<int>() == graph_idx);
        // Parse speed limit
        auto speed_limit_it = j.find("speed_limit");
        if (speed_limit_it != j.end())
          speed_limit = speed_limit_it->get<double>();
        auto dock_name_it = j.find("dock_name");
        if (dock_name_it != j.end())
          dock_name = dock_name_it->get<std::string>();
        auto bidirectional_it = j.find("bidirectional");
        if (bidirectional_it != j.end())
          is_bidirectional = bidirectional_it->get<bool>();

      }
    }
        // Skip if graph_idx is not the equal to the argument
    if (!is_correct_graph)
      continue;
    const auto& lane_feat = feature->GetGeometryRef()->toLineString();
    // Get the points
    double x0 = lane_feat->getX(0);
    double x1 = lane_feat->getX(1);
    double y0 = lane_feat->getY(0);
    double y1 = lane_feat->getY(1);
    double rounded_x0 = std::round(x0 / wp_tolerance) * wp_tolerance;
    double rounded_y0 = std::round(y0 / wp_tolerance) * wp_tolerance;
    double rounded_x1 = std::round(x1 / wp_tolerance) * wp_tolerance;
    double rounded_y1 = std::round(y1 / wp_tolerance) * wp_tolerance;
    auto m0_iter = idx_map[level_idx][rounded_x0].find(rounded_y0);
    if (m0_iter == idx_map[level_idx][rounded_x0].end())
      continue;
    auto m1_iter = idx_map[level_idx][rounded_x1].find(rounded_y1);
    if (m1_iter == idx_map[level_idx][rounded_x1].end())
      continue;
    // TODO waypoint offset
    // Waypoint offset is applied to ensure unique IDs when multiple levels
    // are present
    const std::size_t start_wp = m0_iter->second;
    const std::size_t end_wp = m1_iter->second;

    using Lane = rmf_traffic::agv::Graph::Lane;
    using Event = Lane::Event;
    // TODO(luca) Add lifts, doors, orientation constraints
    rmf_utils::clone_ptr<Event> entry_event;
    rmf_utils::clone_ptr<Event> exit_event;
    if (is_bidirectional)
    {
      // Lane in the opposite direction
      auto& lane = graph.add_lane({end_wp, entry_event},
          {start_wp, exit_event});
      lane.properties().speed_limit(speed_limit);
    }

    // dock_name is only applied to the lane going to the waypoint, not exiting
    const rmf_traffic::Duration duration = std::chrono::seconds(5);
    if (dock_name.has_value())
      entry_event = Event::make(Lane::Dock(dock_name.value(), duration));
    auto& lane = graph.add_lane({start_wp, entry_event},
      {end_wp, exit_event});
    lane.properties().speed_limit(speed_limit);
  }
/*
// Iterate over vertices / waypoints
// Graph params are not used for now
for (const auto& vertex : from.vertices)
{
  const Eigen::Vector2d location{
    vertex.x, vertex.y};
  auto& wp = graph.add_waypoint(from.name, location);
  // Add waypoint name if in the message
  if (vertex.name.size() > 0 && !graph.add_key(vertex.name, wp.index()))
  {
    throw std::runtime_error(
            "Duplicated waypoint name [" + vertex.name + "]");
  }
  for (const auto& param : vertex.params)
  {
    if (param.name == "is_parking_spot")
      wp.set_parking_spot(param.value_bool);
    else if (param.name == "is_holding_point")
      wp.set_holding_point(param.value_bool);
    else if (param.name == "is_passthrough_point")
      wp.set_passthrough_point(param.value_bool);
    else if (param.name == "is_charger")
      wp.set_charger(param.value_bool);
  }
}
// Iterate over edges / lanes
for (const auto& edge : from.edges)
{
  using Lane = rmf_traffic::agv::Graph::Lane;
  using Event = Lane::Event;
  // TODO(luca) Add lifts, doors, orientation constraints
  rmf_utils::clone_ptr<Event> entry_event;
  rmf_utils::clone_ptr<Event> exit_event;
  // Waypoint offset is applied to ensure unique IDs when multiple levels
  // are present
  const std::size_t start_wp = edge.v1_idx + waypoint_offset;
  const std::size_t end_wp = edge.v2_idx + waypoint_offset;
  std::string dock_name;
  std::optional<double> speed_limit;
  for (const auto& param : edge.params)
  {
    if (param.name == "dock_name")
      dock_name = param.value_string;
    if (param.name == "speed_limit")
      speed_limit = param.value_float;
  }
  // dock_name is only applied to the lane going to the waypoint, not exiting
  if (edge.edge_type == edge.EDGE_TYPE_BIDIRECTIONAL)
  {
    auto& lane = graph.add_lane({end_wp, entry_event},
        {start_wp, exit_event});
    lane.properties().speed_limit(speed_limit);
  }

  const rmf_traffic::Duration duration = std::chrono::seconds(5);
  if (dock_name.size() > 0)
    entry_event = Event::make(Lane::Dock(dock_name, duration));
  auto& lane = graph.add_lane({start_wp, entry_event},
    {end_wp, exit_event});
  lane.properties().speed_limit(speed_limit);
}
*/
return graph;
}

} // namespace rmf_traffic_ros2
