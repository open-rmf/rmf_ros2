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

#include <mutex>
#include <rmf_traffic_ros2/schedule/ParticipantRegistry.hpp>
#include <rmf_traffic_ros2/schedule/ParticipantDescription.hpp>
#include "internal_YamlSerialization.hpp"

namespace rmf_traffic_ros2 {
namespace schedule {

//==============================================================================
bool operator!=(
  const rmf_traffic::schedule::ParticipantDescription& p1,
  const rmf_traffic::schedule::ParticipantDescription& p2)
{
  return rmf_traffic_ros2::convert(p1) != rmf_traffic_ros2::convert(p2);
}

//==============================================================================
struct UniqueId
{
  std::string name;
  std::string owner;

  bool operator==(const UniqueId& other) const
  {
    return name == other.name && owner == other.owner;
  }
};

//==============================================================================
struct UniqueIdHasher
{
  std::size_t operator()(UniqueId id) const
  {
    return std::hash<std::string>{} (id.name + id.owner);
  }
};

//==============================================================================
class ParticipantRegistry::Implementation
{
public:
  //===========================================================================
  Implementation(
    std::unique_ptr<AbstractParticipantLogger> logger,
    std::shared_ptr<Database> db)
  : _database(db),
    _logger(std::move(logger))
  {
    _reading_from_log = true;
    while (auto record = _logger->read_next_record())
    {
      execute(*record);
    }
    _reading_from_log = false;
  }

  //===========================================================================
  Registration add_or_retrieve_participant(
    ParticipantDescription new_description)
  {
    std::lock_guard<std::mutex> lock(_mutex);
    const UniqueId key = {new_description.name(), new_description.owner()};
    const auto it = _id_from_name.find(key);

    if (it != _id_from_name.end())
    {
      const auto id = it->second;
      auto& description_entry = _description.at(id);

      // Check if footprint has changed
      if (description_entry != new_description)
      {
        _database->update_description(id, new_description);
        description_entry = new_description;
        write_to_file({AtomicOperation::OpType::Update, new_description});
      }

      return Registration(
        id, _database->itinerary_version(id), _database->last_route_id(id));
    }

    const auto registration = _database->register_participant(new_description);
    _id_from_name[key] = registration.id();
    _description.insert_or_assign(registration.id(), new_description);

    write_to_file({AtomicOperation::OpType::Add, new_description});
    return registration;
  }

  //===========================================================================
  std::optional<ParticipantId> participant_exists(
    const std::string& name,
    const std::string& owner)
  {
    UniqueId key = {name, owner};
    auto id = _id_from_name.find(key);
    if (id == _id_from_name.end())
    {
      return std::nullopt;
    }
    return {id->second};
  }

private:
  //===========================================================================
  void write_to_file(AtomicOperation op)
  {
    if (_reading_from_log)
    {
      // add_or_retrieve_participant will be called when we are reading the
      // file during startup, which will call this function with the entries
      // loaded from the file. We do not want to write those entries back into
      // the file since that's where they came from. So we will skip this step
      // when reading entries from the log.
      return;
    }

    _logger->write_operation(op);
  }

  //==========================================================================
  void execute(AtomicOperation operation)
  {
    if (operation.operation == AtomicOperation::OpType::Add
      || operation.operation == AtomicOperation::OpType::Update)
    {
      add_or_retrieve_participant(operation.description);
    }
  }

  //==========================================================================
  std::unordered_map<UniqueId, ParticipantId, UniqueIdHasher> _id_from_name;
  std::unordered_map<ParticipantId, ParticipantDescription> _description;
  std::shared_ptr<Database> _database;
  std::unique_ptr<AbstractParticipantLogger> _logger;
  std::mutex _mutex;
  bool _reading_from_log = false;
};

//=============================================================================
ParticipantRegistry::ParticipantRegistry(
  std::unique_ptr<AbstractParticipantLogger> logger,
  std::shared_ptr<Database> database)
: _pimpl(rmf_utils::make_unique_impl<Implementation>(
      std::move(logger), database))
{
  // Do nothing
}

//=============================================================================
ParticipantRegistry::Registration
ParticipantRegistry::add_or_retrieve_participant(
  ParticipantDescription description)
{
  return _pimpl->add_or_retrieve_participant(std::move(description));
}

} // namespace schedule
} // namespace rmf_traffic_ros2
