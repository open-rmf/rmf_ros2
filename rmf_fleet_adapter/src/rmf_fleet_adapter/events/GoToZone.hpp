#ifndef SRC__RMF_FLEET_ADAPTER__EVENTS__GOTOZONE_HPP
#define SRC__RMF_FLEET_ADAPTER__EVENTS__GOTOZONE_HPP

#include "../agv/RobotContext.hpp"

#include <optional>
#include <rmf_task_sequence/Event.hpp>
#include <rmf_task_sequence/events/GoToZone.hpp>
#include <rmf_task/events/SimpleEventState.hpp>

namespace rmf_fleet_adapter {
namespace events {

//==============================================================================
class GoToZone : public rmf_task_sequence::Event
{
public:

  using Description = rmf_task_sequence::events::GoToZone::Description;

  static void add(rmf_task_sequence::Event::Initializer& initializer);

  class Standby : public rmf_task_sequence::Event::Standby
  {
  public:

    static std::shared_ptr<Standby> make(
      const AssignIDPtr& id,
      const std::function<rmf_task::State()>& get_state,
      const rmf_task::ConstParametersPtr& parameters,
      const Description& description,
      std::function<void()> update);

    ConstStatePtr state() const final;

    rmf_traffic::Duration duration_estimate() const final;

    ActivePtr begin(
      std::function<void()> checkpoint,
      std::function<void()> finished) final;

  private:

    Standby(Description description);

    Description _description;
    AssignIDPtr _assign_id;
    agv::RobotContextPtr _context;
    rmf_traffic::Duration _time_estimate;
    std::function<void()> _update;
    rmf_task::events::SimpleEventStatePtr _state;
    ActivePtr _active = nullptr;
    std::optional<std::string> _validation_error;
  };
};

} // namespace events
} // namespace rmf_fleet_adapter

#endif // SRC__RMF_FLEET_ADAPTER__EVENTS__GOTOZONE_HPP
