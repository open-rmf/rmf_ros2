#ifndef SRC__RMF_FLEET_ADAPTER__PHASES__ZONEEXIT_HPP
#define SRC__RMF_FLEET_ADAPTER__PHASES__ZONEEXIT_HPP

#include "../LegacyTask.hpp"
#include "../agv/RobotContext.hpp"
#include "rmf_fleet_adapter/StandardNames.hpp"

#include <rmf_zone_msgs/msg/zone_request.hpp>
#include <rmf_zone_msgs/msg/zone_state.hpp>

namespace rmf_fleet_adapter {
namespace phases {

//==============================================================================
class ZoneExit
{
public:
  class PendingPhase;
  class ActivePhase;
};

//==============================================================================
class ZoneExit::PendingPhase : public LegacyTask::PendingPhase
{
public:

  PendingPhase(
    agv::RobotContextPtr context,
    std::string zone_name,
    rmf_traffic::Time expected_finish,
    std::shared_ptr<rmf_traffic::PlanId> plan_id);

  std::shared_ptr<LegacyTask::ActivePhase> begin() override;

  rmf_traffic::Duration estimate_phase_duration() const override;

  const std::string& description() const override;

private:

  agv::RobotContextPtr _context;
  std::string _zone_name;
  rmf_traffic::Time _expected_finish;
  std::shared_ptr<rmf_traffic::PlanId> _plan_id;
  std::string _description;
};

//==============================================================================
class ZoneExit::ActivePhase
: public LegacyTask::ActivePhase,
  public std::enable_shared_from_this<ActivePhase>
{
public:

  static std::shared_ptr<ActivePhase> make(
    agv::RobotContextPtr context,
    std::string zone_name,
    rmf_traffic::Time expected_finish,
    std::shared_ptr<rmf_traffic::PlanId> plan_id);

  const rxcpp::observable<LegacyTask::StatusMsg>& observe() const override;

  rmf_traffic::Duration estimate_remaining_time() const override;

  void emergency_alarm(bool on) override;

  void cancel() override;

  const std::string& description() const override;

private:

  agv::RobotContextPtr _context;
  std::string _zone_name;
  std::string _assigned_waypoint;
  rxcpp::observable<LegacyTask::StatusMsg> _obs;

  rclcpp::Subscription<rmf_zone_msgs::msg::ZoneState>::SharedPtr _state_sub;
  rclcpp::Publisher<rmf_zone_msgs::msg::ZoneRequest>::SharedPtr _request_pub;

  rclcpp::TimerBase::SharedPtr _timeout_timer;
  rclcpp::TimerBase::SharedPtr _delay_timer;
  rmf_traffic::Time _expected_finish;
  std::shared_ptr<rmf_traffic::PlanId> _plan_id;
  std::string _description_text;

  ActivePhase(
    agv::RobotContextPtr context,
    std::string zone_name,
    rmf_traffic::Time expected_finish,
    std::shared_ptr<rmf_traffic::PlanId> plan_id);
  void _init_obs();
};

} // namespace phases
} // namespace rmf_fleet_adapter

#endif // SRC__RMF_FLEET_ADAPTER__PHASES__ZONEEXIT_HPP
