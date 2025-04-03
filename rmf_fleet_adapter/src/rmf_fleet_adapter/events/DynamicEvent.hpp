/*
 * Copyright (C) 2025 Open Source Robotics Foundation
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

#ifndef SRC__RMF_FLEET_ADAPTER__EVENTS__DYNAMICEVENT_HPP
#define SRC__RMF_FLEET_ADAPTER__EVENTS__DYNAMICEVENT_HPP

#include "../agv/RobotContext.hpp"
#include "../agv/internal_FleetUpdateHandle.hpp"

#include <rmf_task_sequence/Event.hpp>
#include <rmf_task/events/SimpleEventState.hpp>

namespace rmf_fleet_adapter {
namespace events {

//==============================================================================
class DynamicEvent : public rmf_task_sequence::Event
{
public:
  class Description;
  using DescriptionPtr = std::shared_ptr<Description>;

  static void add(
    agv::TaskDeserialization& deserialization,
    rmf_task_sequence::Event::InitializerPtr event_initializer);

  class Standby;
  class Active;

};

//==============================================================================
class DynamicEvent::Description : public Event::Description
{
public:

  template<typename... Args>
  static DescriptionPtr make(Args&&... args)
  {
    return std::shared_ptr<Description>(
      new Description(std::forward<Args>(args)...));
  }

  /// Get the original json text of the description. This may contain additional
  /// unparsed parameters set by the user.
  const std::string& json_text() const;

  /// Get the parsed json of the description.
  const nlohmann::json& json() const;

  rmf_task_sequence::Activity::ConstModelPtr make_model(
    rmf_task::State initial_state,
    const rmf_task::Parameters& parameters) const final;

  rmf_task::Header generate_header(
    const rmf_task::State& initial_state,
    const rmf_task::Parameters& parameters) const final;

  const DeserializeJSONPtr<DeserializedEvent>& event_deserializer() const;

  const rmf_task_sequence::Event::ConstInitializerPtr& event_initializer() const;

private:

  Description(
    std::string category,
    std::string detail,
    nlohmann::json description_json,
    rmf_task_sequence::Event::ConstDescriptionPtr estimate,
    DeserializeJSONPtr<DeserializedEvent> event_deserializer,
    rmf_task_sequence::Event::ConstInitializerPtr event_initializer);

  std::string _json_text;
  nlohmann::json _json;
  std::string _category;
  std::string _detail;
  rmf_task_sequence::Event::ConstDescriptionPtr _estimate;
  DeserializeJSONPtr<DeserializedEvent> _event_deserializer;
  rmf_task_sequence::Event::ConstInitializerPtr _event_initializer;
};

//==============================================================================
class DynamicEvent::Standby : public rmf_task_sequence::Event::Standby
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
};

//==============================================================================
class DynamicEvent::Active
  : public rmf_task_sequence::Event::Active,
  public std::enable_shared_from_this<Active>
{
public:

  static std::shared_ptr<Active> make(
    const AssignIDPtr& id,
    agv::RobotContextPtr context,
    Description description,
    rmf_task::events::SimpleEventStatePtr state,
    std::function<void()> update,
    std::function<void()> finished);

  ConstStatePtr state() const final;

  rmf_traffic::Duration remaining_time_estimate() const final;

  Backup backup() const final;

  Resume interrupt(std::function<void()> task_is_interrupted) final;

  void cancel() final;

  void kill() final;

private:

  Active(Description description);

  void _publish_update();

  void _begin_next_event(
    std::shared_ptr<const rmf_task_sequence::Event::Description> child_description);

  Description _description;
  std::shared_ptr<DynamicEventCallbacks> _callbacks;
  AssignIDPtr _assign_ids;
  agv::RobotContextPtr _context;
  rmf_task::events::SimpleEventStatePtr _state;
  std::function<void()> _update;
  std::function<void()> _finished;
  uint32_t _seq;
  std::shared_ptr<void> _current_event_raii;
  rmf_task_sequence::Event::ActivePtr _current_event;
  std::shared_ptr<DynamicEventHandle> _current_handle;
  rmf_task::VersionedString::Reader _string_reader;
  rmf_task::Log::Reader _log_reader;
  bool _interrupted = false;
  bool _cancelled = false;
  std::function<void()> _deferred_event_start;
};

} // namespace events
} // namespace rmf_fleet_adapter

#endif // SRC__RMF_FLEET_ADAPTER__EVENTS__DYNAMICEVENT_HPP
