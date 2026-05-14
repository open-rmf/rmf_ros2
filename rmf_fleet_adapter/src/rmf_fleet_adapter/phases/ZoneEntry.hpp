#ifndef SRC__RMF_FLEET_ADAPTER__PHASES__ZONEENTRY_HPP
#define SRC__RMF_FLEET_ADAPTER__PHASES__ZONEENTRY_HPP

#include "../LegacyTask.hpp"
#include "../agv/RobotContext.hpp"
#include "rmf_fleet_adapter/StandardNames.hpp"

#include <rmf_zone_msgs/msg/zone_request.hpp>
#include <rmf_zone_msgs/msg/zone_state.hpp>

namespace rmf_fleet_adapter {
namespace phases {

//==============================================================================
class ZoneEntry
{
public:
  struct Data
  {
    std::string zone_name;
    rmf_traffic::Time expected_finish;
    std::shared_ptr<rmf_traffic::schedule::Itinerary> resume_itinerary;
    std::shared_ptr<rmf_traffic::PlanId> plan_id;
  };

  class PendingPhase;
  class ActivePhase;
};

//==============================================================================
class ZoneEntry::PendingPhase : public LegacyTask::PendingPhase
{
public:

  PendingPhase(
    agv::RobotContextPtr context,
    std::string zone_name,
    rmf_traffic::Time expected_finish,
    std::shared_ptr<rmf_traffic::schedule::Itinerary> resume_itinerary,
    std::shared_ptr<rmf_traffic::PlanId> plan_id);

  std::shared_ptr<LegacyTask::ActivePhase> begin() override;

  rmf_traffic::Duration estimate_phase_duration() const override;

  const std::string& description() const override;

private:

  agv::RobotContextPtr _context;
  Data _data;
  std::string _description;
};

//==============================================================================
class ZoneEntry::ActivePhase
: public LegacyTask::ActivePhase,
  public std::enable_shared_from_this<ActivePhase>
{
public:

  static std::shared_ptr<ActivePhase> make(
    agv::RobotContextPtr context, Data data);

  const rxcpp::observable<LegacyTask::StatusMsg>& observe() const override;

  rmf_traffic::Duration estimate_remaining_time() const override;

  void emergency_alarm(bool on) override;

  void cancel() override;

  const std::string& description() const override;

private:

  agv::RobotContextPtr _context;
  Data _data;
  rxcpp::observable<LegacyTask::StatusMsg> _obs;

  rclcpp::Subscription<rmf_zone_msgs::msg::ZoneState>::SharedPtr _state_sub;
  rclcpp::Publisher<rmf_zone_msgs::msg::ZoneRequest>::SharedPtr _request_pub;

  std::string _current_request_id;
  bool _has_pending_request = false;

  rclcpp::TimerBase::SharedPtr _delay_timer;

  std::string _description_text;

  ActivePhase(agv::RobotContextPtr context, Data data);
  void _init_obs();
};

} // namespace phases
} // namespace rmf_fleet_adapter

#endif // SRC__RMF_FLEET_ADAPTER__PHASES__ZONEENTRY_HPP
