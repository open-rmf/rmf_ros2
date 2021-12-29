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

#include <rmf_traffic/geometry/Circle.hpp>
#include <rmf_traffic_ros2/schedule/ParticipantRegistry.hpp>
#include <filesystem>
#include <rmf_utils/catch.hpp>
#include <fstream>
#include <cstdio>

#include "../../src/rmf_traffic_ros2/schedule/internal_YamlSerialization.hpp"

using namespace rmf_traffic_ros2::schedule;
SCENARIO("Test idempotency of shape type")
{
  YAML::Node node;

  // Check "None"
  REQUIRE_THROWS(serialize_shape_type(
      rmf_traffic_msgs::msg::ConvexShape::NONE));

  // Check "Box"
  auto serialized =
    serialize_shape_type(rmf_traffic_msgs::msg::ConvexShape::BOX);
  node["type"] = serialized;
  CHECK(shape_type(node["type"]) == rmf_traffic_msgs::msg::ConvexShape::BOX);

  // Check "Circle"
  serialized = serialize_shape_type(rmf_traffic_msgs::msg::ConvexShape::CIRCLE);
  node["type"] = serialized;
  CHECK(shape_type(node["type"]) == rmf_traffic_msgs::msg::ConvexShape::CIRCLE);

  REQUIRE_THROWS(serialize_shape_type(42));
  node["type"] = "not a shape";
  REQUIRE_THROWS(shape_type(node["type"]));
}

namespace rmf_traffic_ros2 {
namespace schedule {

bool operator==(
  const AtomicOperation op1,
  const AtomicOperation op2)
{
  return op1.operation == op2.operation
    && op1.description == op2.description;
}

}
}

SCENARIO("Test idempotency of ParticipantDescription.")
{
  //Lets create a bunch of participants
  const auto shape = rmf_traffic::geometry::make_final_convex<
    rmf_traffic::geometry::Circle>(1.0);

  rmf_traffic::schedule::ParticipantDescription p1(
    "participant 1",
    "test_Participant",
    rmf_traffic::schedule::ParticipantDescription::Rx::Responsive,
    rmf_traffic::Profile{shape});

  auto serialized = serialize(p1);
  auto result = participant_description(serialized);
  REQUIRE(p1 == result);
}

class TestOperationLogger : public AbstractParticipantLogger
{
public:

  TestOperationLogger(std::vector<AtomicOperation>* journal)
  {
    _counter = 0;
    _journal = journal;
  }

  void write_operation(AtomicOperation operation) override
  {
    _journal->push_back(operation);
  }

  std::optional<AtomicOperation> read_next_record() override
  {
    if (_counter >= _journal->size())
    {
      return std::nullopt;
    }
    return {(*_journal)[_counter++]};
  }

private:
  std::vector<AtomicOperation>* _journal;
  std::size_t _counter;
};

SCENARIO("Participant registry restores participants from logger")
{
  using Database = rmf_traffic::schedule::Database;

  //Lets create a bunch of participants
  const auto shape = rmf_traffic::geometry::make_final_convex<
    rmf_traffic::geometry::Circle>(1.0);

  rmf_traffic::schedule::ParticipantDescription p1(
    "participant 1",
    "test_Participant",
    rmf_traffic::schedule::ParticipantDescription::Rx::Responsive,
    rmf_traffic::Profile{shape});

  rmf_traffic::schedule::ParticipantDescription p2(
    "participant 2",
    "test_Participant",
    rmf_traffic::schedule::ParticipantDescription::Rx::Responsive,
    rmf_traffic::Profile{shape});

  rmf_traffic::schedule::ParticipantDescription p3(
    "participant 3",
    "test_Participant",
    rmf_traffic::schedule::ParticipantDescription::Rx::Responsive,
    rmf_traffic::Profile{shape});

  GIVEN("A stubbed out logger")
  {
    std::vector<AtomicOperation> journal;
    auto logger = std::make_unique<TestOperationLogger>(&journal);
    WHEN("Creating a new DB without errors")
    {
      auto db1 = std::make_shared<Database>();
      ParticipantRegistry registry1(std::move(logger), db1);
      const auto participant_id1 = registry1.add_or_retrieve_participant(p1);
      const auto participant_id2 = registry1.add_or_retrieve_participant(p2);
      const auto participant_id3 = registry1.add_or_retrieve_participant(p3);

      std::vector<AtomicOperation> journal_old = journal;

      THEN("Restoring DB")
      {
        auto db2 = std::make_shared<Database>();
        auto logger2 = std::make_unique<TestOperationLogger>(&journal);
        ParticipantRegistry registry2(std::move(logger2), db2);
        auto restored_participants = db2->participant_ids();
        REQUIRE(restored_participants.count(participant_id1.id()) > 0);
        REQUIRE(restored_participants.count(participant_id2.id()) > 0);
        REQUIRE(restored_participants.count(participant_id3.id()) > 0);
        REQUIRE(restored_participants.size() == 3);

        // Checks that the logs have not accidentally grown or shrunk or been
        // mutated.
        REQUIRE(journal.size() == 3);
        for (std::size_t i = 0; i < journal.size(); i++)
          REQUIRE(journal[i] == journal_old[i]);

        auto _p1 = db2->get_participant(participant_id1.id());
        auto _p2 = db2->get_participant(participant_id2.id());
        auto _p3 = db2->get_participant(participant_id3.id());

        REQUIRE(*_p1 == p1);
        REQUIRE(*_p2 == p2);
        REQUIRE(*_p3 == p3);
      }
    }
  }
}

SCENARIO("Test file logger")
{
  if (std::filesystem::exists("test_yamllogger.yaml"))
  {
    std::remove("test_yamllogger.yaml");
  }

  const auto shape = rmf_traffic::geometry::make_final_convex<
    rmf_traffic::geometry::Circle>(1.0);

  rmf_traffic::schedule::ParticipantDescription p1(
    "participant 1",
    "test_Participant",
    rmf_traffic::schedule::ParticipantDescription::Rx::Responsive,
    rmf_traffic::Profile{shape});

  rmf_traffic::schedule::ParticipantDescription p2(
    "participant 2",
    "test_Participant",
    rmf_traffic::schedule::ParticipantDescription::Rx::Responsive,
    rmf_traffic::Profile{shape});

  rmf_traffic::schedule::ParticipantDescription p3(
    "participant 3",
    "test_Participant",
    rmf_traffic::schedule::ParticipantDescription::Rx::Responsive,
    rmf_traffic::Profile{shape});

  GIVEN("non-existant file")
  {
    WHEN("Storing three records")
    {
      YamlLogger logger1("test_yamllogger.yaml");
      logger1.write_operation({AtomicOperation::OpType::Add, p1});
      logger1.write_operation({AtomicOperation::OpType::Add, p2});
      THEN("Able to retrieve 3 records")
      {
        YamlLogger logger2("test_yamllogger.yaml");
        std::vector<AtomicOperation> expected = {
          {AtomicOperation::OpType::Add, p1},
          {AtomicOperation::OpType::Add, p2},
        };

        std::size_t i = 0;
        while (auto record = logger2.read_next_record())
        {
          REQUIRE(i < expected.size());
          REQUIRE(expected[i] == *record);
          i++;
        }
        REQUIRE(i == expected.size());
      }
    }
  }

  GIVEN("corrupt file that is in no way YAML")
  {
    std::ofstream invalid_yaml;
    invalid_yaml.open("test_yamllogger.yaml", std::ofstream::out);
    invalid_yaml << "\"rubbish yaml^";
    invalid_yaml.close();
    WHEN("Parsing")
    {
      THEN("throws exception")
      {
        REQUIRE_THROWS(new YamlLogger("test_yamllogger.yaml"));
      }
    }
  }

  GIVEN("a yaml file that does not fit ")
  {

    WHEN("the file is not a sequence")
    {
      YAML::Node node;
      node["some_yaml"] = "invalid data";
      std::ofstream invalid_yaml;
      invalid_yaml.open("test_yamllogger.yaml", std::ofstream::out);
      YAML::Emitter emitter;
      emitter << node;
      invalid_yaml << emitter.c_str();
      invalid_yaml.close();
      THEN("throw an error upon coinstruction")
      {
        REQUIRE_THROWS(std::make_shared<YamlLogger>("test_yamllogger.yaml"));
      }
    }

    WHEN("the file is a YAML sequence but filled with rubbish")
    {
      YAML::Node node;
      std::ofstream invalid_yaml;
      node["some_yaml"] = "invalid data";
      YAML::Emitter emmiter;
      emmiter << YAML::BeginSeq;
      emmiter << node;
      emmiter << YAML::EndSeq;
      invalid_yaml.open("test_yamllogger.yaml", std::ofstream::out);
      invalid_yaml << emmiter.c_str();
      invalid_yaml.close();
      THEN("throw an error upon connecting to database")
      {
        auto logger = std::make_unique<YamlLogger>("test_yamllogger.yaml");
        auto db = std::make_shared<Database>();
        REQUIRE_THROWS(new ParticipantRegistry(std::move(logger), db));
      }
    }
  }
}
