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

#include <chrono>

#include <rmf_traffic_ros2/schedule/internal_Node.hpp>
#include <rmf_traffic_ros2/schedule/Query.hpp>
#include <rmf_traffic_ros2/StandardNames.hpp>

#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

class MissingQueryScheduleNode : public rmf_traffic_ros2::schedule::ScheduleNode
{
public:
  MissingQueryScheduleNode(const rclcpp::NodeOptions& options)
    : ScheduleNode(0, options)
  {
    timer = create_wall_timer(35s, [this]() -> void
      {
        if (have_queries)
        {
          RCLCPP_WARN(get_logger(), "Committing suicide");
          std::exit(0);
        }
      });
  }

  void broadcast_queries() override
  {
    ScheduleQueries msg;

    bool is_first = true;
    for (const auto& [query_id, info] : registered_queries)
    {
      if (is_first)
      {
        // Don't broadcast the first query so the fail-over node doesn't know about
        // it (creates a missing query when the replacement node starts)
        is_first = false;
        continue;
      }
      msg.ids.push_back(query_id);

      const rmf_traffic::schedule::Query& original = info.query;
      ScheduleQuery query = rmf_traffic_ros2::convert(original);
      msg.queries.push_back(query);
    }
    queries_info_pub->publish(msg);

    have_queries = true;
  }

  bool have_queries = false;
  rclcpp::TimerBase::SharedPtr timer;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MissingQueryScheduleNode>(rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
