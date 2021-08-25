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

#ifndef RMF_TRAFFIC_ROS2__PARTICIPANT_REGISTRY_HPP
#define RMF_TRAFFIC_ROS2__PARTICIPANT_REGISTRY_HPP


#include <rmf_traffic_ros2/Profile.hpp>
#include <rmf_traffic_ros2/geometry/ConvexShape.hpp>
#include <rmf_traffic/schedule/Database.hpp>
#include <string>
#include <yaml-cpp/yaml.h>
#include <unordered_map>

namespace rmf_traffic_ros2 {
namespace schedule {

using Database = rmf_traffic::schedule::Database;
using ParticipantId = rmf_traffic::schedule::ParticipantId;
using ParticipantDescription = rmf_traffic::schedule::ParticipantDescription;

//=============================================================================
/// This records a single operation on the database class
struct AtomicOperation
{
  enum class OpType : uint8_t
  {
    Add = 0,
    Update = 1
  };
  OpType operation;
  ParticipantDescription description;
};

//=============================================================================
/// This is the base class for the persistence logger.
class AbstractParticipantLogger
{
public:
  /// Called when we wish to commit an operation to disk
  /// \param[in] operation
  virtual void write_operation(AtomicOperation operation) = 0;

  /// Called when we wish to read the next record up during initiallization.
  /// \returns a std::nullopt when we have exhausted all records.
  virtual std::optional<AtomicOperation> read_next_record() = 0;

  virtual ~AbstractParticipantLogger() = default;
};

//=============================================================================
/// YAML logger class. Logs everything to YAML buffers on disk
class YamlLogger : public AbstractParticipantLogger
{
public:
  /// Constructor
  /// Loads and logs to the specified file.
  ///
  /// \throws YAML::ParserException if there is an error in the syntax of log
  /// file.
  ///
  /// \throws YAML::BadFile if there are problems with reading the file.
  ///
  /// \throws std::filesystem_error if there is no permission to create the
  /// directory.
  YamlLogger(std::string filename);

  /// See AbstractParticipantLogger
  void write_operation(AtomicOperation operation) override;

  /// See AbstractParticipantLogger
  /// \throws std::runtime_error if there was an error in the logfile.
  std::optional<AtomicOperation> read_next_record() override;

  class Implementation;
private:
  rmf_utils::unique_impl_ptr<Implementation> _pimpl;
};

//=============================================================================
/// Adds a persistance layer to the participant ids. This allows the scheduler
/// to restart without the need to restart fleet adapters.
/// Internally, this class implements a an append only journal. This makes it
/// independent of any id generation inside the database, as long as the said
/// database id generation algorithm is deterministic.
class ParticipantRegistry
{
public:
  /// Constructor
  ///
  /// \param[in] logger
  ///   The logging implementation to use for recording registration.
  ///
  /// \param[in] database
  ///   The database that will register the participants.
  ParticipantRegistry(
    std::unique_ptr<AbstractParticipantLogger> logger,
    std::shared_ptr<Database> database);

  using Registration = rmf_traffic::schedule::Writer::Registration;

  /// Adds a participant or retrieves its ID if it was already added in the past
  ///
  /// \param[in] description
  ///   The description of the participant that one wishes to register.
  ///
  /// \returns ParticipantId of the participant
  Registration add_or_retrieve_participant(ParticipantDescription description);

  class Implementation;
private:
  rmf_utils::unique_impl_ptr<Implementation> _pimpl;
};

} // namespace schedule
} // namespace rmf_traffic_ros2

#endif
