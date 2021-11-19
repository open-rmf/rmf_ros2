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

#include <proj.h>
#include <nlohmann/json.hpp>

#include <rmf_traffic_ros2/agv/Graph.hpp>

namespace rmf_traffic_ros2 {

// Usage map[level_idx][truncated_x][truncated_y] = id;
// Truncation is to 1e-3 meters
using CoordsIdxHashMap = std::unordered_map<std::size_t, std::unordered_map<
  double, std::unordered_map<double, std::size_t>>>;

// local helper function to factor json parsing code for both
// the compressed and uncompressed case
static rmf_traffic::agv::Graph json_to_graph(
  const std::vector<uint8_t>& json_doc,
  const int graph_idx,
  const double wp_tolerance);

bool decompress_gzip(const std::vector<uint8_t>& in, std::vector<uint8_t>& out)
{
  z_stream strm;
  memset(&strm, 0, sizeof(strm));
  strm.zalloc = Z_NULL;
  strm.zfree = Z_NULL;
  strm.opaque = Z_NULL;
  strm.avail_in = 0;
  strm.next_in = Z_NULL;
  const int inflate_init_ret = inflateInit2(&strm, 15 + 32);
  if (inflate_init_ret != Z_OK)
  {
    printf("error in inflateInit2()\n");
    return false;
  }

  const size_t READ_CHUNK_SIZE = 128 * 1024;
  size_t read_pos = 0;

  const size_t OUT_CHUNK_SIZE = 128 * 1024;
  std::vector<uint8_t> inflate_buf(OUT_CHUNK_SIZE);

  do
  {
    if (read_pos + READ_CHUNK_SIZE < in.size())
      strm.avail_in = READ_CHUNK_SIZE;
    else
      strm.avail_in = in.size() - read_pos;
    // printf("read %d\n", (int)strm.avail_in);
    strm.next_in = (unsigned char *)&in[read_pos];
    read_pos += strm.avail_in;

    int inflate_ret = 0;
    do
    {
      strm.avail_out = inflate_buf.size();
      strm.next_out = &inflate_buf[0];
      inflate_ret = inflate(&strm, Z_NO_FLUSH);
      if (inflate_ret == Z_NEED_DICT ||
          inflate_ret == Z_DATA_ERROR ||
          inflate_ret == Z_MEM_ERROR)
      {
        printf("unrecoverable zlib inflate error\n");
        inflateEnd(&strm);
        return false;
      }
      const int n_have = inflate_buf.size() - strm.avail_out;
      out.insert(
        out.end(),
        inflate_buf.begin(),
        inflate_buf.begin() + n_have);
      /*
      printf("write %d, output size: %d\n",
        (int)n_have,
        (int)out.size());
      */
    } while (strm.avail_out == 0);
    
    if (inflate_ret == Z_STREAM_END)
      break;
  } while (read_pos < in.size());

  printf("inflated: %d -> %d\n", (int)in.size(), (int)out.size());

  return true;
}

//==============================================================================
rmf_traffic::agv::Graph convert(const rmf_site_map_msgs::msg::SiteMap& from,
  int graph_idx, double wp_tolerance)
{
  rmf_traffic::agv::Graph graph;
  if (from.encoding == from.MAP_DATA_GEOJSON)
  {
    printf("converting GeoJSON map\n");
    return json_to_graph(from.data, graph_idx, wp_tolerance);
  }
  else if (from.encoding == from.MAP_DATA_GEOJSON_GZ)
  {
    printf("converting compressed GeoJSON map\n");
    std::vector<uint8_t> uncompressed;
    if (!decompress_gzip(from.data, uncompressed))
      return graph;  // failed to decompress gzip, cannot proceed
    return json_to_graph(uncompressed, graph_idx, wp_tolerance);
  }
  else
    return graph;  // unexpected encoding
}

rmf_traffic::agv::Graph json_to_graph(
  const std::vector<uint8_t>& json_doc,
  const int graph_idx,
  const double wp_tolerance)
{
  rmf_traffic::agv::Graph graph;
  printf("json_to_graph with doc length %d\n", (int)json_doc.size());
  nlohmann::json j = nlohmann::json::parse(json_doc);
  printf("parsed %d entries in json\n", (int)j.size());
  //auto graph_idx_it = j.find("graph_idx");

  if (!j.contains("preferred_crs") || !j["preferred_crs"].is_string()) {
    printf("GeoJSON does not contain top-level preferred_crs key!\n");
    return graph;
  }
  const std::string preferred_crs = j["preferred_crs"];
  printf("preferred_crs: [%s]\n", preferred_crs.c_str());

  if (!j.contains("features") || !j["features"].is_array()) {
    printf("GeoJSON does not contain top-level features array!\n");
    return graph;
  }

  CoordsIdxHashMap idx_map;

  // spin through features and find all vertices
  PJ_CONTEXT *proj_context = proj_context_create();
  PJ *projector = proj_create_crs_to_crs(
    proj_context,
    "EPSG:4326",  // aka WGS84, always used in GeoJSON
    preferred_crs.c_str(),
    NULL);
  if (!projector)
  {
    printf("unable to create coordinate projector!\n");
    return graph;
  }

  for (const auto& feature : j["features"])
  {
    const std::string feature_type = feature["feature_type"];
    if (feature_type != "nav_vertex")
      continue;

    // sanity check the object structure
    if (!feature.contains("properties") || !feature["properties"].is_object())
      continue;
    if (!feature.contains("geometry") || !feature["geometry"].is_object())
      continue;
    const auto& geom = feature["geometry"];
    if (!geom.contains("type") || !geom["type"].is_string())
      continue;
    if (geom["type"] != "Point")
      continue;
    if (!geom.contains("coordinates") || !geom["coordinates"].is_array())
      continue;
    if (geom["coordinates"].size() < 2)
      continue;

    // GeoJSON always encodes coordinates as (lon, lat)
    const double lon = geom["coordinates"][0];
    const double lat = geom["coordinates"][1];
    
    std::string name;
    if (feature["properties"].contains("name"))
      name = feature["properties"]["name"];

    int level_idx = 0;
    if (feature["properties"].contains("level_idx"))
      level_idx = feature["properties"]["level_idx"];

    // todo: parse other parameters here

    const PJ_COORD wgs84_coord = proj_coord(lat, lon, 0, 0);
    const PJ_COORD p = proj_trans(projector, PJ_FWD, wgs84_coord);

    // not sure why the coordinate-flip is required, but... it is.
    const double easting = p.enu.n;
    const double northing = p.enu.e;

    const Eigen::Vector2d location{easting, northing};

    // TODO map name
    std::string map_name("test");
    auto& wp = graph.add_waypoint(map_name, location);

    if (name.size() > 0 && !graph.add_key(name, wp.index()))
    {
      throw std::runtime_error(
              "Duplicated waypoint name [" + name + "]");
    }

    double rounded_x = std::round(easting / wp_tolerance) * wp_tolerance;
    double rounded_y = std::round(northing / wp_tolerance) * wp_tolerance;
    idx_map[level_idx][rounded_x][rounded_y] = wp.index();

    //printf("vertex name: [%s] coords: (%.6f, %.6f) -> (%.2f, %.2f)\n",
    //  name.c_str(), lon, lat, easting, northing);
  }
  proj_context_destroy(proj_context);
  return graph;
}

#if 0
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
#endif

} // namespace rmf_traffic_ros2
