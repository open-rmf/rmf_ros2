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
// Truncation is to 1e-3 meters by default
using CoordsIdxHashMap = std::unordered_map<std::size_t, std::unordered_map<
      double, std::unordered_map<double, std::size_t>>>;

// local helper function to factor json parsing code for both
// the compressed and uncompressed case
static rmf_traffic::agv::Graph json_to_graph(
  const std::vector<uint8_t>& json_doc,
  const int graph_idx,
  const double wp_tolerance);

std::optional<std::vector<uint8_t>> decompress_gzip(
  const std::vector<uint8_t>& in)
{
  std::vector<uint8_t> out;
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
    std::cout << "error in inflateInit2()" << std::endl;
    return std::nullopt;
  }

  const std::size_t READ_CHUNK_SIZE = 128 * 1024;
  std::size_t read_pos = 0;

  std::vector<uint8_t> inflate_buf(READ_CHUNK_SIZE);

  do
  {
    if (read_pos + READ_CHUNK_SIZE < in.size())
      strm.avail_in = READ_CHUNK_SIZE;
    else
      strm.avail_in = in.size() - read_pos;

    strm.next_in = (unsigned char*)&in[read_pos];
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
        std::cout << "unrecoverable zlib inflate error" << std::endl;
        inflateEnd(&strm);
        return std::nullopt;
      }
      const int n_have = inflate_buf.size() - strm.avail_out;
      out.insert(
        out.end(),
        inflate_buf.begin(),
        inflate_buf.begin() + n_have);
    } while (strm.avail_out == 0);

    if (inflate_ret == Z_STREAM_END)
      break;
  } while (read_pos < in.size());

  std::cout << "inflated: " << in.size() << " -> " << out.size() << std::endl;

  return {out};
}

//==============================================================================
rmf_traffic::agv::Graph convert(const rmf_site_map_msgs::msg::SiteMap& from,
  int graph_idx, double wp_tolerance)
{
  rmf_traffic::agv::Graph graph;
  if (from.encoding == from.MAP_DATA_GEOJSON)
  {
    std::cout << "converting GeoJSON map" << std::endl;
    return json_to_graph(from.data, graph_idx, wp_tolerance);
  }
  else if (from.encoding == from.MAP_DATA_GEOJSON_GZ)
  {
    std::cout << "converting compressed GeoJSON map" << std::endl;
    const auto uncompressed = decompress_gzip(from.data);
    if (!uncompressed.has_value())
      return graph;
    return json_to_graph(uncompressed.value(), graph_idx, wp_tolerance);
  }
  else
  {
    std::cout << "unexpected encoding value: " << from.encoding << std::endl;
    return graph;  // unexpected encoding
  }
}

rmf_traffic::agv::Graph json_to_graph(
  const std::vector<uint8_t>& json_doc,
  const int graph_idx,
  const double wp_tolerance)
{
  rmf_traffic::agv::Graph graph;
  std::cout << "json_to_graph with doc length " << json_doc.size() << std::endl;
  nlohmann::json j = nlohmann::json::parse(json_doc);
  std::cout << "parsed " << j.size() << "entries in json" << std::endl;

  const auto preferred_crs_it = j.find("preferred_crs");
  if (preferred_crs_it == j.end() || !preferred_crs_it->is_string())
  {
    std::cout << "GeoJSON does not contain top-level preferred_crs key!" <<
      std::endl;
    return graph;
  }
  const std::string preferred_crs = *preferred_crs_it;
  std::cout << "preferred_crs: " << preferred_crs << std::endl;

  if (!j.contains("features") || !j["features"].is_array())
  {
    std::cout << "GeoJSON does not contain top-level features array!" <<
      std::endl;
    return graph;
  }

  const auto site_name_it = j.find("site_name");
  if (site_name_it == j.end() || !site_name_it->is_string())
  {
    std::cout << "Site name not found in map" << std::endl;
    return graph;
  }
  const std::string site_name = *site_name_it;

  CoordsIdxHashMap idx_map;

  // spin through features and find all vertices
  PJ_CONTEXT* proj_context = proj_context_create();
  PJ* projector = proj_create_crs_to_crs(
    proj_context,
    "EPSG:4326",  // aka WGS84, always used in GeoJSON
    preferred_crs.c_str(),
    NULL);
  if (!projector)
  {
    std::cout << "unable to create coordinate projector!" << std::endl;
    return graph;
  }

  for (const auto& feature : j["features"])
  {
    const std::string feature_type = feature["feature_type"];
    if (feature_type != "rmf_vertex")
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

    std::string name = feature["properties"].value("name", "");

    int level_idx = feature["properties"].value("level_idx", 0);

    // todo: parse other parameters here

    const PJ_COORD wgs84_coord = proj_coord(lat, lon, 0, 0);
    const PJ_COORD p = proj_trans(projector, PJ_FWD, wgs84_coord);

    // not sure why the coordinate-flip is required, but... it is.
    const double easting = p.enu.n;
    const double northing = p.enu.e;

    const Eigen::Vector2d location{easting, northing};

    auto& wp = graph.add_waypoint(site_name, location);

    if (name.size() > 0 && !graph.add_key(name, wp.index()))
    {
      throw std::runtime_error(
              "Duplicated waypoint name [" + name + "]");
    }

    double rounded_x = std::round(easting / wp_tolerance) * wp_tolerance;
    double rounded_y = std::round(northing / wp_tolerance) * wp_tolerance;
    idx_map[level_idx][rounded_x][rounded_y] = wp.index();

    // Set waypoint properties
    if (feature["properties"].contains("is_holding_point"))
      wp.set_holding_point(feature["properties"]["is_holding_point"]);
    if (feature["properties"].contains("is_passthrough_point"))
      wp.set_passthrough_point(feature["properties"]["is_passthrough_point"]);
    if (feature["properties"].contains("is_parking_spot"))
      wp.set_parking_spot(feature["properties"]["is_parking_spot"]);
    if (feature["properties"].contains("is_charger"))
      wp.set_charger(feature["properties"]["is_charger"]);
  }

  // now spin through the features again, looking for lanes
  for (const auto& feature : j["features"])
  {
    const std::string feature_type = feature["feature_type"];
    if (feature_type != "rmf_lane")
      continue;

    if (!feature.contains("geometry") || !feature["geometry"].is_object())
      continue;
    const auto& geom = feature["geometry"];
    if (!geom.contains("type") || !geom["type"].is_string())
      continue;
    if (geom["type"] != "LineString")
      continue;
    if (!geom.contains("coordinates") || !geom["coordinates"].is_array())
      continue;
    if (geom["coordinates"].size() < 2)
      continue;

    if (feature["properties"].contains("graph_idx"))
    {
      const int lane_graph_idx = feature["properties"]["graph_idx"];
      if (lane_graph_idx != graph_idx)
        continue;
    }

    int level_idx = feature["properties"].value("level_idx", 0);
    bool is_bidirectional = feature["properties"].value("bidirectional", false);

    std::optional<double> speed_limit;
    if (feature["properties"].contains("speed_limit"))
      speed_limit = feature["properties"]["speed_limit"];

    std::optional<std::string> dock_name;
    if (feature["properties"].contains("dock_name"))
      dock_name = feature["properties"]["dock_name"];

    const double lon_0 = geom["coordinates"][0][0];
    const double lat_0 = geom["coordinates"][0][1];
    const double lon_1 = geom["coordinates"][1][0];
    const double lat_1 = geom["coordinates"][1][1];

    const PJ_COORD wgs84_coord_0 = proj_coord(lat_0, lon_0, 0, 0);
    const PJ_COORD wgs84_coord_1 = proj_coord(lat_1, lon_1, 0, 0);
    const PJ_COORD p0 = proj_trans(projector, PJ_FWD, wgs84_coord_0);
    const PJ_COORD p1 = proj_trans(projector, PJ_FWD, wgs84_coord_1);

    // not sure why the coordinate-flip is required, but... it is.
    // maybe can use proj_normalize_for_visualization someday?
    const double x0 = p0.enu.n;
    const double y0 = p0.enu.e;
    const double x1 = p1.enu.n;
    const double y1 = p1.enu.e;

    const double rounded_x0 = std::round(x0 / wp_tolerance) * wp_tolerance;
    const double rounded_y0 = std::round(y0 / wp_tolerance) * wp_tolerance;
    const double rounded_x1 = std::round(x1 / wp_tolerance) * wp_tolerance;
    const double rounded_y1 = std::round(y1 / wp_tolerance) * wp_tolerance;

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

  proj_destroy(projector);
  proj_context_destroy(proj_context);
  return graph;
}

} // namespace rmf_traffic_ros2
