/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

#ifndef SRC__RMF_FLEET_ADAPTER__PHASES__INGESTITEM_HPP
#define SRC__RMF_FLEET_ADAPTER__PHASES__INGESTITEM_HPP

#include "RxOperators.hpp"
#include "../LegacyTask.hpp"
#include "../agv/RobotContext.hpp"
#include "rmf_fleet_adapter/StandardNames.hpp"

#include <rmf_rxcpp/Transport.hpp>
#include <rmf_ingestor_msgs/msg/ingestor_state.hpp>
#include <rmf_ingestor_msgs/msg/ingestor_result.hpp>
#include <rmf_ingestor_msgs/msg/ingestor_request.hpp>

namespace rmf_fleet_adapter {
namespace phases {

struct IngestItem
{
  class ActivePhase : public LegacyTask::ActivePhase,
    public std::enable_shared_from_this<ActivePhase>
  {
  public:

    static std::shared_ptr<ActivePhase> make(
      agv::RobotContextPtr context,
      std::string request_guid,
      std::string target,
      std::string transporter_type,
      std::vector<rmf_ingestor_msgs::msg::IngestorRequestItem> items);

    const rxcpp::observable<LegacyTask::StatusMsg>& observe() const override;

    rmf_traffic::Duration estimate_remaining_time() const override;

    void emergency_alarm(bool on) override;

    void cancel() override;

    const std::string& description() const override;

  private:

    agv::RobotContextPtr _context;
    std::string _request_guid;
    std::string _target;
    std::string _transporter_type;
    std::vector<rmf_ingestor_msgs::msg::IngestorRequestItem> _items;
    std::string _description;
    rxcpp::observable<LegacyTask::StatusMsg> _obs;
    rclcpp::TimerBase::SharedPtr _timer;
    bool _request_acknowledged = false;
    builtin_interfaces::msg::Time _last_msg;

    ActivePhase(
      agv::RobotContextPtr context,
      std::string request_guid,
      std::string target,
      std::string transporter_type,
      std::vector<rmf_ingestor_msgs::msg::IngestorRequestItem> items);

    void _init_obs();

    LegacyTask::StatusMsg _get_status(
      const rmf_ingestor_msgs::msg::IngestorResult::SharedPtr& ingestor_result,
      const rmf_ingestor_msgs::msg::IngestorState::SharedPtr& ingestor_state);

    void _do_publish();
  };

  class PendingPhase : public LegacyTask::PendingPhase
  {
  public:

    PendingPhase(
      agv::RobotContextPtr context,
      std::string request_guid,
      std::string target,
      std::string transporter_type,
      std::vector<rmf_ingestor_msgs::msg::IngestorRequestItem> items);

    std::shared_ptr<LegacyTask::ActivePhase> begin() override;

    rmf_traffic::Duration estimate_phase_duration() const override;

    const std::string& description() const override;

  private:

    agv::RobotContextPtr _context;
    std::string _request_guid;
    std::string _target;
    std::string _transporter_type;
    std::vector<rmf_ingestor_msgs::msg::IngestorRequestItem> _items;
    std::string _description;
  };
};

} // namespace phases
} // namespace rmf_fleet_adapter

#endif // SRC__RMF_FLEET_ADAPTER__PHASES__INGESTITEM_HPP
