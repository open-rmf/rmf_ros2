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
#include "internal_YamlSerialization.hpp"

namespace rmf_traffic_ros2 {
namespace schedule {

//=============================================================================
struct UniqueId
{
  std::string name;
  std::string owner;

  bool operator==(const UniqueId &other) const
  {
    return name == other.name && owner == other.owner;
  }
};

//=============================================================================
struct UniqueIdHasher
{
  std::size_t operator()(UniqueId id) const
  {
    return std::hash<std::string>{}(id.name + id.owner);
  }
};

//=============================================================================
class ParticipantRegistry::Implementation
{
public:
  //===========================================================================
  Implementation(
    std::unique_ptr<AbstractParticipantLogger>  logger, 
    std::shared_ptr<Database> db):
    _database(db),
    _logger(std::move(logger))
  { 
    init();
  }

  //===========================================================================
  Registration add_or_retrieve_participant(ParticipantDescription description)
  {
    std::lock_guard<std::mutex> lock(_mutex);
    const UniqueId key = {description.name(), description.owner()};
    const auto it = _id_from_name.find(key);
    if (it != _id_from_name.end())
    {
      const auto id = it->second;
      return Registration(
        id, _database->itinerary_version(id), _database->last_route_id(id));
    }

    const auto registration = _database->register_participant(description);
    _id_from_name[key] = registration.id();

    write_to_file({AtomicOperation::OpType::Add, description});
    return registration;
  }

  //===========================================================================
  std::optional<ParticipantId> participant_exists(
    std::string name,
    std::string owner)
  {
    UniqueId key = {name, owner};
    auto id = _id_from_name.find(key);
    if(id == _id_from_name.end())
    {
      return std::nullopt;
    }
    return {id->second};
  }
  
private:
  //===========================================================================
  void write_to_file(AtomicOperation op)
  {
    //if restoring in progress don't update the log
    if(_currently_restoring) return;
    
    _logger->write_operation(op);
  }
 
  //===========================================================================
  void init()
  {
    _currently_restoring = true;
    while(auto record = _logger->read_next_record())
    {
      execute(*record);
    }
    _currently_restoring= false;
  }

  //==========================================================================
  void execute(AtomicOperation operation)
  {
    if(operation.operation == AtomicOperation::OpType::Add)
    {
      add_or_retrieve_participant(operation.description);
    }
  }

  //==========================================================================
  bool _currently_restoring;
  std::unordered_map<UniqueId, ParticipantId, UniqueIdHasher> _id_from_name;
  std::shared_ptr<Database> _database; 
  std::unique_ptr<AbstractParticipantLogger> _logger;
  std::mutex _mutex;
};

//=============================================================================
ParticipantRegistry::ParticipantRegistry(
  std::unique_ptr<AbstractParticipantLogger> logger,
  std::shared_ptr<Database> database)
:_pimpl(rmf_utils::make_unique_impl<Implementation>(
  std::move(logger), database))
{
  //Do nothing
}

//=============================================================================
ParticipantRegistry::Registration
ParticipantRegistry::add_or_retrieve_participant(
  ParticipantDescription description)
{
  return _pimpl->add_or_retrieve_participant(description);
}

} // namespace schedule
} // namespace rmf_traffic_ros2
