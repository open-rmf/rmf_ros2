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

#include <rmf_utils/catch.hpp>

#include <rmf_traffic_ros2/agv/Graph.hpp>
#include <rmf_traffic_ros2/schedule/ParticipantRegistry.hpp>

static auto make_graph_node(const double x, const double y,
  const std::string& name,
  const std::vector<rmf_building_map_msgs::msg::Param>& params)
{
  rmf_building_map_msgs::msg::GraphNode node;
  node.x = x;
  node.y = y;
  node.name = name;
  for (const auto& param : params)
    node.params.push_back(param);
  return node;
}

static auto make_bool_param(const std::string& name, const bool val)
{
  rmf_building_map_msgs::msg::Param param;
  param.name = name;
  param.type = param.TYPE_BOOL;
  param.value_bool = val;
  return param;
} 

static auto make_string_param(const std::string& name, const std::string& val)
{
  rmf_building_map_msgs::msg::Param param;
  param.name = name;
  param.type = param.TYPE_STRING;
  param.value_string = val;
  return param;
}

SCENARIO("Test conversion from rmf_building_map_msgs to rmf_traffic")
{
  GIVEN("a graph with two nodes")
  {
    rmf_building_map_msgs::msg::Graph input_msg;
    // Populate input graph
    input_msg.name = "test_graph";
    rmf_building_map_msgs::msg::GraphNode node;
    input_msg.vertices.push_back(
      make_graph_node(10, 20, "wp_0", {
        make_bool_param("is_parking_spot", true),
        make_bool_param("is_holding_point", true),
        make_string_param("dock_name", "dock_1")
      }));
    input_msg.vertices.push_back(
      make_graph_node(30, 40, "wp_1", {
        make_bool_param("is_passthrough_point", true),
        make_bool_param("is_charger", true)
      }));
    // A node without a key
    input_msg.vertices.push_back(
      make_graph_node(50, 60, "", {}));
    rmf_building_map_msgs::msg::GraphEdge edge;
    edge.v1_idx = 0;
    edge.v2_idx = 1;
    WHEN("the lane is not bidirectional")
    {
      THEN("graph has only one lane")
      {
        edge.edge_type = edge.EDGE_TYPE_UNIDIRECTIONAL;
        input_msg.edges = {edge};
        auto res = rmf_traffic_ros2::convert(input_msg);
        REQUIRE(res.num_lanes() == 1);
        auto lane = res.get_lane(0);
        CHECK(lane.entry().waypoint_index() == 0);
        CHECK(lane.exit().waypoint_index() == 1);
      }
    }
    WHEN("the lane is bidirectional")
    {
      THEN("graph has two lanes")
      {
        edge.edge_type = edge.EDGE_TYPE_BIDIRECTIONAL;
        input_msg.edges = {edge};
        auto res = rmf_traffic_ros2::convert(input_msg);
        REQUIRE(res.num_lanes() == 2);
        auto lane = res.get_lane(0);
        // Expect one to be 1 and one to be 0, hence the XOR
        CHECK(
          (lane.entry().waypoint_index() ^ lane.exit().waypoint_index()) == 1);
        lane = res.get_lane(1);
        CHECK(
          (lane.entry().waypoint_index() ^ lane.exit().waypoint_index()) == 1);
        CHECK(lane.entry().waypoint_index() !=
          res.get_lane(0).entry().waypoint_index());

        // Check for the dock event in the first lane
        auto entry_event = lane.entry().event();
        CHECK(entry_event != nullptr);
      }
    }
    THEN("output graph has two nodes with right properties")
    {
      auto res = rmf_traffic_ros2::convert(input_msg);
      auto keys = res.keys();
      // Only two nodes had a key but three nodes
      CHECK(keys.size() == 2);
      CHECK(res.num_waypoints() == 3);
      // Check the nodes properties
      REQUIRE(res.find_waypoint("wp_0") != nullptr);
      REQUIRE(res.find_waypoint("wp_1") != nullptr);
      REQUIRE(res.find_waypoint("wp_2") == nullptr);
      auto wp = res.find_waypoint("wp_0");
      CHECK(wp->is_parking_spot() == true);
      CHECK(wp->is_holding_point() == true);
      CHECK(wp->is_passthrough_point() == false);
      CHECK(wp->is_charger() == false);
      wp = res.find_waypoint("wp_1");
      CHECK(wp->is_parking_spot() == false);
      CHECK(wp->is_holding_point() == false);
      CHECK(wp->is_passthrough_point() == true);
      CHECK(wp->is_charger() == true);
    }
  }
}
