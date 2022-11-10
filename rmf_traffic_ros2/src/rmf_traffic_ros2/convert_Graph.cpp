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

#include <rmf_building_map_msgs/msg/graph_node.hpp>
#include <rmf_building_map_msgs/msg/graph_edge.hpp>
#include <rmf_building_map_msgs/msg/param.hpp>

#include <unordered_set>

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

//==============================================================================
std::optional<rmf_traffic::agv::Graph> convert(
  const rmf_building_map_msgs::msg::Graph& navgraph)
{
  using GraphParamMsg = rmf_building_map_msgs::msg::Param;
  using Lane = rmf_traffic::agv::Graph::Lane;
  using Event = Lane::Event;

  rmf_traffic::agv::Graph graph;
  std::unordered_set<std::size_t> added_waypoints = {};

  for (const auto& v : navgraph.vertices)
  {
    const std::string wp_name = v.name;
    std::string map_name = "";
    bool is_holding_point = false;
    bool is_passthrough_point = false;
    bool is_parking_spot = false;
    bool is_charger = false;
    Eigen::Vector2d loc = {v.x, v.y};
    for (const auto& p : v.params)
    {
      if (p.name == "is_holding_point")
      {
        is_holding_point = p.value_bool;
      }
      else if (p.name == "is_passthrough_point")
      {
        is_passthrough_point = p.value_bool;
      }
      else if (p.name == "is_parking_spot")
      {
        is_parking_spot = p.value_bool;
      }
      else if (p.name == "is_charger")
      {
        is_charger = p.value_bool;
      }
      else if (p.name == "map_name")
      {
        map_name = p.value_string;
      }
    }
    // The map_name field is essential for a Graph::Waypoint
    if (map_name.empty())
      return std::nullopt;
    graph.add_waypoint(map_name, loc)
    .set_holding_point(is_holding_point)
    .set_passthrough_point(is_passthrough_point)
    .set_parking_spot(is_parking_spot)
    .set_charger(is_charger);
    const auto wp_index = graph.num_waypoints() - 1;
    if (!graph.set_key(wp_name, wp_index))
      return std::nullopt;
    added_waypoints.insert(wp_index);
  }
  for (const auto& e : navgraph.edges)
  {
    if (added_waypoints.find(e.v1_idx) == added_waypoints.end() ||
      added_waypoints.find(e.v2_idx) == added_waypoints.end())
    {
      return std::nullopt;
    }

    // Maps to generate events and properties
    std::unordered_map<std::string, GraphParamMsg> entry_params;
    std::unordered_map<std::string, GraphParamMsg> exit_params;
    std::unordered_map<std::string, GraphParamMsg> other_params;
    for (const auto& param : e.params)
    {
      if (param.name.find("entry_") != std::string::npos)
      {
        entry_params.insert({param.name.substr(6), param});
      }
      else if (param.name.find("exit_") != std::string::npos)
      {
        exit_params.insert({param.name.substr(5), param});
      }
      else
      {
        other_params.insert({param.name, param});
      }
    }

    rmf_utils::clone_ptr<Event> entry_event = nullptr;
    rmf_utils::clone_ptr<rmf_traffic::agv::Graph::OrientationConstraint>
    entry_constraint = nullptr;

    rmf_utils::clone_ptr<Event> exit_event = nullptr;
    rmf_utils::clone_ptr<rmf_traffic::agv::Graph::OrientationConstraint>
    exit_constraint = nullptr;

    auto set_event_and_constraint =
      [](
      const std::unordered_map<std::string, GraphParamMsg>& params,
      rmf_utils::clone_ptr<Event>& event_to_set,
      rmf_utils::clone_ptr<rmf_traffic::agv::Graph::OrientationConstraint>&
      constraint_to_set)
      {
        using Lane = rmf_traffic::agv::Graph::Lane;
        using Constraint = rmf_traffic::agv::Graph::OrientationConstraint;
        using Event = Lane::Event;
        using DoorOpen = Lane::DoorOpen;
        using DoorClose = Lane::DoorClose;
        using LiftSessionBegin = Lane::LiftSessionBegin;
        using LiftDoorOpen = Lane::LiftDoorOpen;
        using LiftSessionEnd = Lane::LiftSessionEnd;
        using LiftMove = Lane::LiftMove;
        using Dock = Lane::Dock;
        using Wait = Lane::Wait;

        if (params.empty())
          return;

        // Dock
        if (params.find("dock_name") != params.end() &&
          params.find("dock_duration") != params.end())
        {
          std::string name = params.at("dock_name").value_string;
          rmf_traffic::Duration duration = std::chrono::nanoseconds(
            static_cast<uint64_t>(params.at("dock_duration").value_float));
          event_to_set = Event::make(Dock(std::move(name), duration));
        }
        // DoorOpen
        else if (params.find("door_open_name") != params.end() &&
          params.find("door_open_duration") != params.end())
        {
          std::string name = params.at("door_open_name").value_string;
          rmf_traffic::Duration duration = std::chrono::nanoseconds(
            static_cast<uint64_t>(params.at("door_open_duration").value_float));
          event_to_set = Event::make(DoorOpen(std::move(name), duration));
        }
        // DoorClose
        else if (params.find("door_close_name") != params.end() &&
          params.find("door_close_duration") != params.end())
        {
          std::string name = params.at("door_close_name").value_string;
          rmf_traffic::Duration duration = std::chrono::nanoseconds(
            static_cast<uint64_t>(params.at("door_close_duration").value_float));
          event_to_set = Event::make(DoorClose(std::move(name), duration));
        }
        // LiftSessionBegin
        else if (params.find("lift_end_lift_name") != params.end() &&
          params.find("lift_end_floor_name") != params.end() &&
          params.find("lift_end_duration") != params.end())
        {
          std::string lift_name = params.at("lift_end_lift_name").value_string;
          std::string floor_name =
            params.at("lift_end_floor_name").value_string;
          rmf_traffic::Duration duration = std::chrono::nanoseconds(
            static_cast<uint64_t>(params.at("lift_end_duration").value_float));
          event_to_set = Event::make(LiftSessionBegin(
                std::move(lift_name),
                std::move(floor_name),
                duration));
        }
        // LiftMove
        else if (params.find("lift_move_lift_name") != params.end() &&
          params.find("lift_move_floor_name") != params.end() &&
          params.find("lift_move_duration") != params.end())
        {
          std::string lift_name = params.at("lift_move_lift_name").value_string;
          std::string floor_name =
            params.at("lift_move_floor_name").value_string;
          rmf_traffic::Duration duration = std::chrono::nanoseconds(
            static_cast<uint64_t>(params.at("lift_move_duration").value_float));
          event_to_set = Event::make(LiftMove(
                std::move(lift_name),
                std::move(floor_name),
                duration));
        }
        // LiftDoorOpen
        else if (params.find("lift_door_open_lift_name") != params.end() &&
          params.find("lift_door_open_floor_name") != params.end() &&
          params.find("lift_door_open_duration") != params.end())
        {
          std::string lift_name =
            params.at("lift_door_open_lift_name").value_string;
          std::string floor_name =
            params.at("lift_door_open_floor_name").value_string;
          rmf_traffic::Duration duration = std::chrono::nanoseconds(
            static_cast<uint64_t>(params.at("lift_door_open_duration").
            value_float));
          event_to_set = Event::make(LiftDoorOpen(
                std::move(lift_name),
                std::move(floor_name),
                duration));
        }
        // LiftSessionEnd
        else if (params.find("lift_end_lift_name") != params.end() &&
          params.find("lift_end_floor_name") != params.end() &&
          params.find("lift_end_duration") != params.end())
        {
          std::string lift_name = params.at("lift_end_lift_name").value_string;
          std::string floor_name =
            params.at("lift_end_floor_name").value_string;
          rmf_traffic::Duration duration = std::chrono::nanoseconds(
            static_cast<uint64_t>(params.at("lift_end_duration").value_float));
          event_to_set = Event::make(LiftSessionEnd(
                std::move(lift_name),
                std::move(floor_name),
                duration));
        }
        // Dock
        else if (params.find("wait_duration") != params.end())
        {
          rmf_traffic::Duration duration = std::chrono::nanoseconds(
            static_cast<uint64_t>(params.at("wait_duration").value_float));
          event_to_set = Event::make(Wait(duration));
        }
        //OrientationConstraint
        else if (params.find("orientation_constraint") != params.end())
        {
          const auto& dir_str =
            params.at("orientation_constraint").value_string;
          const Constraint::Direction dir = dir_str ==
            "forward" ? Constraint::Direction::Forward :
            Constraint::Direction::Backward;
          constraint_to_set = Constraint::make(
            dir, Eigen::Vector2d::UnitX());
        }
        else
        {
          return;
        }
      };

    set_event_and_constraint(entry_params, entry_event, entry_constraint);
    set_event_and_constraint(exit_params, exit_event, exit_constraint);

    // Deserialize other properties
    auto lane_properties = rmf_traffic::agv::Graph::Lane::Properties();
    for (const auto& [name, param] : other_params)
    {
      if (name == "speed_limit" &&
        param.type == param.TYPE_DOUBLE &&
        param.value_float > 0.0)
      {
        lane_properties.speed_limit(param.value_float);
      }
    }

    graph.add_lane(
      {e.v1_idx, entry_event, entry_constraint},
      {e.v2_idx, exit_event, exit_constraint},
      std::move(lane_properties));
  }

  return graph;
}

//==============================================================================
// An event factory that will help serialize the event
class EventPhaseFactory : public rmf_traffic::agv::Graph::Lane::Executor
{
public:

  using Lane = rmf_traffic::agv::Graph::Lane;
  using GraphParamMsg = rmf_building_map_msgs::msg::Param;

  EventPhaseFactory(
    const std::string& prefix,
    std::vector<GraphParamMsg>& edge_params)
  : _prefix(std::move(prefix)),
    _edge_params(edge_params)
  {
    // Do nothing
  }

  void execute(const Dock& dock) final
  {
    _edge_params.emplace_back(rmf_building_map_msgs::build<GraphParamMsg>()
      .name(_prefix + "_dock_name")
      .type(GraphParamMsg::TYPE_STRING)
      .value_int(0)
      .value_float(0)
      .value_string(dock.dock_name())
      .value_bool(false));

    _edge_params.emplace_back(rmf_building_map_msgs::build<GraphParamMsg>()
      .name(_prefix + "_dock_duration")
      .type(GraphParamMsg::TYPE_INT)
      .value_int(0)
      .value_float(dock.duration().count())
      .value_string("")
      .value_bool(false));
  }

  void execute(const DoorOpen& open) final
  {
    _edge_params.emplace_back(rmf_building_map_msgs::build<GraphParamMsg>()
      .name(_prefix + "_door_open_name")
      .type(GraphParamMsg::TYPE_STRING)
      .value_int(0)
      .value_float(0)
      .value_string(open.name())
      .value_bool(false));

    _edge_params.emplace_back(rmf_building_map_msgs::build<GraphParamMsg>()
      .name(_prefix + "_door_open_duration")
      .type(GraphParamMsg::TYPE_INT)
      .value_int(0)
      .value_float(open.duration().count())
      .value_string("")
      .value_bool(false));
  }

  void execute(const DoorClose& close) final
  {
    _edge_params.emplace_back(rmf_building_map_msgs::build<GraphParamMsg>()
      .name(_prefix + "_door_close_name")
      .type(GraphParamMsg::TYPE_STRING)
      .value_int(0)
      .value_float(0)
      .value_string(close.name())
      .value_bool(false));

    _edge_params.emplace_back(rmf_building_map_msgs::build<GraphParamMsg>()
      .name(_prefix + "_door_close_duration")
      .type(GraphParamMsg::TYPE_INT)
      .value_int(0)
      .value_float(close.duration().count())
      .value_string("")
      .value_bool(false));
  }

  void execute(const LiftSessionBegin& open) final
  {
    _edge_params.emplace_back(rmf_building_map_msgs::build<GraphParamMsg>()
      .name(_prefix + "_lift_end_lift_name")
      .type(GraphParamMsg::TYPE_STRING)
      .value_int(0)
      .value_float(0)
      .value_string(open.lift_name())
      .value_bool(false));

    _edge_params.emplace_back(rmf_building_map_msgs::build<GraphParamMsg>()
      .name(_prefix + "_lift_end_floor_name")
      .type(GraphParamMsg::TYPE_STRING)
      .value_int(0)
      .value_float(0)
      .value_string(open.floor_name())
      .value_bool(false));

    _edge_params.emplace_back(rmf_building_map_msgs::build<GraphParamMsg>()
      .name(_prefix + "_lift_end_duration")
      .type(GraphParamMsg::TYPE_INT)
      .value_int(0)
      .value_float(open.duration().count())
      .value_string("")
      .value_bool(false));
  }

  void execute(const LiftMove& move) final
  {
    _edge_params.emplace_back(rmf_building_map_msgs::build<GraphParamMsg>()
      .name(_prefix + "_lift_move_lift_name")
      .type(GraphParamMsg::TYPE_STRING)
      .value_int(0)
      .value_float(0)
      .value_string(move.lift_name())
      .value_bool(false));

    _edge_params.emplace_back(rmf_building_map_msgs::build<GraphParamMsg>()
      .name(_prefix + "_lift_move_floor_name")
      .type(GraphParamMsg::TYPE_STRING)
      .value_int(0)
      .value_float(0)
      .value_string(move.floor_name())
      .value_bool(false));

    _edge_params.emplace_back(rmf_building_map_msgs::build<GraphParamMsg>()
      .name(_prefix + "_lift_move_duration")
      .type(GraphParamMsg::TYPE_INT)
      .value_int(0)
      .value_float(move.duration().count())
      .value_string("")
      .value_bool(false));
  }

  void execute(const LiftDoorOpen& open) final
  {
    _edge_params.emplace_back(rmf_building_map_msgs::build<GraphParamMsg>()
      .name(_prefix + "_lift_door_open_lift_name")
      .type(GraphParamMsg::TYPE_STRING)
      .value_int(0)
      .value_float(0)
      .value_string(open.lift_name())
      .value_bool(false));

    _edge_params.emplace_back(rmf_building_map_msgs::build<GraphParamMsg>()
      .name(_prefix + "_lift_door_open_floor_name")
      .type(GraphParamMsg::TYPE_STRING)
      .value_int(0)
      .value_float(0)
      .value_string(open.floor_name())
      .value_bool(false));

    _edge_params.emplace_back(rmf_building_map_msgs::build<GraphParamMsg>()
      .name(_prefix + "_lift_door_open_duration")
      .type(GraphParamMsg::TYPE_INT)
      .value_int(0)
      .value_float(open.duration().count())
      .value_string("")
      .value_bool(false));
  }

  void execute(const LiftSessionEnd& close) final
  {
    _edge_params.emplace_back(rmf_building_map_msgs::build<GraphParamMsg>()
      .name(_prefix + "_lift_end_lift_name")
      .type(GraphParamMsg::TYPE_STRING)
      .value_int(0)
      .value_float(0)
      .value_string(close.lift_name())
      .value_bool(false));

    _edge_params.emplace_back(rmf_building_map_msgs::build<GraphParamMsg>()
      .name(_prefix + "_lift_end_floor_name")
      .type(GraphParamMsg::TYPE_STRING)
      .value_int(0)
      .value_float(0)
      .value_string(close.floor_name())
      .value_bool(false));

    _edge_params.emplace_back(rmf_building_map_msgs::build<GraphParamMsg>()
      .name(_prefix + "_lift_end_duration")
      .type(GraphParamMsg::TYPE_INT)
      .value_int(0)
      .value_float(close.duration().count())
      .value_string("")
      .value_bool(false));
  }

  void execute(const Wait& wait) final
  {
    _edge_params.emplace_back(rmf_building_map_msgs::build<GraphParamMsg>()
      .name(_prefix + "_wait_duration")
      .type(GraphParamMsg::TYPE_INT)
      .value_int(0)
      .value_float(wait.duration().count())
      .value_string("")
      .value_bool(false));
  }

private:
  std::string _prefix;
  std::vector<GraphParamMsg>& _edge_params;
};

//==============================================================================
std::unique_ptr<rmf_building_map_msgs::msg::Graph> convert(
  const rmf_traffic::agv::Graph& graph, const std::string& fleet_name)
{
  using GraphMsg = rmf_building_map_msgs::msg::Graph;
  using GraphNodeMsg = rmf_building_map_msgs::msg::GraphNode;
  using GraphEdgeMsg = rmf_building_map_msgs::msg::GraphEdge;
  using GraphParamMsg = rmf_building_map_msgs::msg::Param;

  if (fleet_name.empty())
    return nullptr;

  const std::size_t n_waypoints = graph.num_waypoints();
  const std::size_t n_lanes = graph.num_lanes();

  if (n_waypoints == 0 || n_lanes == 0)
    return nullptr;

  std::vector<GraphNodeMsg> vertices;
  std::vector<GraphEdgeMsg> edges;
  // Populate vertices
  for (std::size_t i = 0; i < n_waypoints; ++i)
  {
    const auto& wp = graph.get_waypoint(i);
    const auto& loc = wp.get_location();
    std::vector<GraphParamMsg> params;
    params.emplace_back(rmf_building_map_msgs::build<GraphParamMsg>()
      .name("map_name")
      .type(GraphParamMsg::TYPE_STRING)
      .value_int(0)
      .value_float(0.0)
      .value_string(wp.get_map_name())
      .value_bool(false));
    params.emplace_back(rmf_building_map_msgs::build<GraphParamMsg>()
      .name("is_holding_point")
      .type(GraphParamMsg::TYPE_BOOL)
      .value_int(0)
      .value_float(0.0)
      .value_string("")
      .value_bool(wp.is_holding_point()));
    params.emplace_back(rmf_building_map_msgs::build<GraphParamMsg>()
      .name("is_passthrough_point")
      .type(GraphParamMsg::TYPE_BOOL)
      .value_int(0)
      .value_float(0.0)
      .value_string("")
      .value_bool(wp.is_passthrough_point()));
    params.emplace_back(rmf_building_map_msgs::build<GraphParamMsg>()
      .name("is_parking_spot")
      .type(GraphParamMsg::TYPE_BOOL)
      .value_int(0)
      .value_float(0.0)
      .value_string("")
      .value_bool(wp.is_parking_spot()));
    params.emplace_back(rmf_building_map_msgs::build<GraphParamMsg>()
      .name("is_charger")
      .type(GraphParamMsg::TYPE_BOOL)
      .value_int(0)
      .value_float(0.0)
      .value_string("")
      .value_bool(wp.is_charger()));
    const std::string wp_name = wp.name() ? *wp.name() : "";
    vertices.emplace_back(rmf_building_map_msgs::build<GraphNodeMsg>()
      .x(loc[0])
      .y(loc[1])
      .name(wp_name)
      .params(std::move(params)));
  }
  // Populate edges
  for (std::size_t i = 0; i < n_lanes; ++i)
  {
    const auto& lane = graph.get_lane(i);
    // All lanes in rmf_traffic::agv::Graph are unidirectional
    std::vector<GraphParamMsg> params;
    const auto& properties = graph.get_lane(i).properties();
    if (properties.speed_limit().has_value())
    {
      params.emplace_back(rmf_building_map_msgs::build<GraphParamMsg>()
        .name("speed_limit")
        .type(GraphParamMsg::TYPE_DOUBLE)
        .value_int(0)
        .value_float(properties.speed_limit().value())
        .value_string("")
        .value_bool(false));
    }
    // Serialize events for this lane
    const auto entry_event = lane.entry().event();
    if (entry_event != nullptr)
    {
      EventPhaseFactory factory("entry", params);
      entry_event->execute(factory);
    }
    const auto* entry_orientation = lane.entry().orientation_constraint();
    if (entry_orientation != nullptr)
    {
      const auto& forward = Eigen::Vector2d::UnitX();
      Eigen::Vector3d pos = {1.0, 0.0, 0.0};
      entry_orientation->apply(pos, forward);
      std::string dir = std::abs((pos[2] - 0.0)) <
        1e-3 ? "forward" : "backward";
      params.emplace_back(rmf_building_map_msgs::build<GraphParamMsg>()
        .name("entry_orientation_constraint")
        .type(GraphParamMsg::TYPE_STRING)
        .value_int(0)
        .value_float(0.0)
        .value_string(std::move(dir))
        .value_bool(false));
    }
    const auto exit_event = lane.exit().event();
    if (exit_event != nullptr)
    {
      EventPhaseFactory factory("exit", params);
      entry_event->execute(factory);
    }
    const auto* exit_orientation = lane.entry().orientation_constraint();
    if (exit_orientation != nullptr)
    {
      const auto& forward = Eigen::Vector2d::UnitX();
      Eigen::Vector3d pos = {1.0, 0.0, 0.0};
      exit_orientation->apply(pos, forward);
      std::string dir = std::abs((pos[2] - 0.0)) <
        1e-3 ? "forward" : "backward";
      params.emplace_back(rmf_building_map_msgs::build<GraphParamMsg>()
        .name("exit_orientation_constraint")
        .type(GraphParamMsg::TYPE_STRING)
        .value_int(0)
        .value_float(0.0)
        .value_string(std::move(dir))
        .value_bool(false));
    }
    // Add vertices for this lane
    edges.emplace_back(
      rmf_building_map_msgs::build<GraphEdgeMsg>()
      .v1_idx(lane.entry().waypoint_index())
      .v2_idx(lane.exit().waypoint_index())
      .params(std::move(params))
      .edge_type(GraphEdgeMsg::EDGE_TYPE_UNIDIRECTIONAL)
    );
  }

  std::unique_ptr<GraphMsg> msg = std::make_unique<GraphMsg>(
    rmf_building_map_msgs::build<GraphMsg>()
    .name(fleet_name)
    .vertices(std::move(vertices))
    .edges(std::move(edges))
    .params({})
  );

  return msg;
}

} // namespace rmf_traffic_ros2
