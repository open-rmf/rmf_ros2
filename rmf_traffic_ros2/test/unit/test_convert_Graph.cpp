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

#include <fstream>

#include <rmf_utils/catch.hpp>

#include <rmf_traffic_ros2/agv/Graph.hpp>
#include <rmf_traffic_ros2/schedule/ParticipantRegistry.hpp>

const std::string MAP_PATH = TEST_RESOURCES_DIR "/office_map.geojson";

static auto make_map_message(const std::string& map_path)
{
  std::ifstream f(map_path);
  std::stringstream string_buffer;
  string_buffer << f.rdbuf();
  const std::string contents = string_buffer.str();
  // Make the message
  rmf_site_map_msgs::msg::SiteMap msg;
  msg.encoding = msg.MAP_DATA_GEOJSON;
  msg.data = {contents.begin(), contents.end()};
  return msg;
}

SCENARIO("Test conversion from rmf_building_map_msgs to rmf_traffic")
{
  GIVEN("A sample map from an office demo world")
  {
    auto graph = rmf_traffic_ros2::convert(make_map_message(MAP_PATH), 0);
    THEN("Map has all the graph waypoints and lanes")
    {
      // 68 waypoints in the map
      CHECK(graph.num_waypoints() == 68);
      // 7 waypoints are named
      CHECK(graph.keys().size() == 7);
      // 64 lanes (32 bidirectional)
      CHECK(graph.num_lanes() == 64);
    }
    THEN("Waypoint properties are parsed correctly")
    {
      const auto pantry_wp = graph.find_waypoint("tinyRobot1_charger");
      const auto non_existing_wp = graph.find_waypoint("non_existing_wp");
      CHECK(non_existing_wp == nullptr);
      REQUIRE(pantry_wp != nullptr);
      CHECK(pantry_wp->get_map_name() == "building");
      // TODO check location
      //CHECK(pantry_wp->get_location() == {1, 2});
      CHECK(pantry_wp->is_holding_point() == true);
      CHECK(pantry_wp->is_passthrough_point() == false);
      CHECK(pantry_wp->is_parking_spot() == true);
      CHECK(pantry_wp->is_charger() == true);
    }
    THEN("Lane connectivity")
    {
      const auto coe_wp = graph.find_waypoint("coe");
      REQUIRE(coe_wp != nullptr);
      const std::size_t wp_idx = coe_wp->index();
      const auto lanes_from_coe = graph.lanes_from(wp_idx);
      REQUIRE(lanes_from_coe.size() == 1);
      // TODO check this goes to the correct lane
      // TODO lane properties + waypoint events
    }
  }

}

/*
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
        make_bool_param("is_holding_point", true)
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
    edge.params.push_back(make_string_param("dock_name", "dock_1"));
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
        int num_events = 0;
        if (lane.entry().event() != nullptr)
          ++num_events;
        // Expect one to be 1 and one to be 0, hence the XOR
        CHECK(
          (lane.entry().waypoint_index() ^ lane.exit().waypoint_index()) == 1);
        lane = res.get_lane(1);
        if (lane.entry().event() != nullptr)
          ++num_events;
        CHECK(
          (lane.entry().waypoint_index() ^ lane.exit().waypoint_index()) == 1);
        CHECK(lane.entry().waypoint_index() !=
          res.get_lane(0).entry().waypoint_index());

        // Check for the dock event in the first lane
        CHECK(num_events == 1);
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
*/
