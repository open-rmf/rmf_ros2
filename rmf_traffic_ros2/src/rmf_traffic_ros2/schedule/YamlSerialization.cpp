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

#include "internal_YamlSerialization.hpp"

namespace rmf_traffic_ros2 {
namespace schedule {

namespace {
const std::string BoxKey = "Box";
const std::string CircleKey = "Circle";
const std::string TypeKey = "type";
const std::string IndexKey = "index";
const std::string FootprintKey = "footprint";
const std::string VicinityKey = "vicinity";
const std::string NameKey = "name";
const std::string ShapeContextKey = "shape_context";
const std::string GroupKey = "group";
const std::string ResponsivenessKey = "responsiveness";
const std::string ResponsiveKey = "Responsive";
const std::string UnresponsiveKey = "Unresponsive";
const std::string ProfileKey = "profile";
const std::string OperationKey = "operation";
const std::string ParticipantDescriptionKey = "participant_description";
const std::string AddOperationKey = "Add";
} // anonymous namespace

//==============================================================================
ParticipantDescription::Rx responsiveness(YAML::Node node)
{
  auto response = node.as<std::string>();
  if (response == UnresponsiveKey)
    return ParticipantDescription::Rx::Unresponsive;
  if (response == ResponsiveKey)
    return ParticipantDescription::Rx::Responsive;

  throw YAML::ParserException(node.Mark(),
          "[" + ResponsivenessKey + "] field contains unknown identifier: "
          + response);
}

//==============================================================================
uint8_t shape_type(YAML::Node node)
{
  auto type = node.as<std::string>();

  if (type == BoxKey)
    return rmf_traffic_msgs::msg::ConvexShape::BOX;

  if (type == CircleKey)
    return rmf_traffic_msgs::msg::ConvexShape::CIRCLE;

  throw YAML::ParserException(node.Mark(),
          "Shape type must be one of [" + BoxKey + "], [" + CircleKey + "]");
}

//==============================================================================
rmf_traffic_msgs::msg::ConvexShape convex_shape(YAML::Node node)
{
  if (!node.IsMap())
    throw YAML::ParserException(node.Mark(),
            "Shape information should be a map");

  if (!node[TypeKey])
    throw YAML::ParserException(node.Mark(),
            "Shape information missing [" + TypeKey + "]");

  if (!node[IndexKey])
    throw YAML::ParserException(node.Mark(),
            "Shape information missing [" + IndexKey + "]");

  rmf_traffic_msgs::msg::ConvexShape shape;
  shape.type = shape_type(node[TypeKey]);
  shape.index = node[IndexKey].as<uint16_t>();
  return shape;
}

//==============================================================================
rmf_traffic_msgs::msg::ConvexShapeContext shape_context(YAML::Node node)
{
  //For now since only circles are supported, so I'm just going to store their
  //Radii. In future this should change.
  rmf_traffic_msgs::msg::ConvexShapeContext shape_context;
  if (!node.IsSequence())
    throw YAML::ParserException(node.Mark(),
            "[" + ShapeContextKey + "] should be a list");

  for (auto item : node)
  {
    double radius = item.as<double>();
    rmf_traffic_msgs::msg::Circle circle;
    circle.radius = radius;
    shape_context.circles.push_back(circle);
  }

  return shape_context;
}

//==============================================================================
rmf_traffic::Profile profile(YAML::Node node)
{
  if (!node.IsMap())
    throw YAML::ParserException(node.Mark(),
            "Profile information should be a map");

  if (!node[FootprintKey])
    throw YAML::ParserException(node.Mark(),
            "Profile information missing [" + FootprintKey + "]");

  if (!node[VicinityKey])
    throw YAML::ParserException(node.Mark(),
            "Profile information missing [" + VicinityKey + "]");

  if (!node[ShapeContextKey])
    throw YAML::ParserException(node.Mark(),
            "Profile information missing [" + ShapeContextKey + "]");

  rmf_traffic_msgs::msg::Profile profile_msg;
  auto footprint = convex_shape(node[FootprintKey]);
  auto vicinity = convex_shape(node[VicinityKey]);
  auto context = shape_context(node[ShapeContextKey]);

  profile_msg.footprint = footprint;
  profile_msg.vicinity = vicinity;
  profile_msg.shape_context = context;

  rmf_traffic::Profile profile = convert(profile_msg);
  return profile;
}

//==============================================================================
ParticipantDescription participant_description(YAML::Node node)
{

  if (!node.IsMap())
  {
    throw YAML::ParserException(node.Mark(),
            "Participant description should be a map field.");
  }

  if (!node[NameKey])
  {
    throw YAML::ParserException(node.Mark(),
            "Participant description missing [" + NameKey + "] field.");
  }

  if (!node[GroupKey])
  {
    throw YAML::ParserException(node.Mark(),
            "Participant description missing [" + GroupKey + "] field.");
  }

  if (!node[ResponsivenessKey])
  {
    throw YAML::ParserException(node.Mark(),
            "Participant description missing [" + ResponsivenessKey + "] field"
    );
  }

  if (!node[ProfileKey])
  {
    throw YAML::ParserException(node.Mark(),
            "Participant description missing [" + ProfileKey + "] field");
  }

  std::string name = node[NameKey].as<std::string>();
  std::string group = node[GroupKey].as<std::string>();
  ParticipantDescription final_desc(
    name,
    group,
    responsiveness(node[ResponsivenessKey]),
    profile(node[ProfileKey])
  );

  return final_desc;
}

//==============================================================================
AtomicOperation atomic_operation(YAML::Node node)
{
  if (!node.IsMap())
    throw YAML::ParserException(node.Mark(),
            "Malformatted operation: Expected a map");

  AtomicOperation::OpType op_type;

  if (!node[OperationKey])
    throw YAML::ParserException(node.Mark(), "Expected an operation field.");

  const std::string& operation = node[OperationKey].as<std::string>();
  if (operation == AddOperationKey)
  {
    op_type = AtomicOperation::OpType::Add;
  }
  else
  {
    throw YAML::ParserException(node.Mark(), "Invalid operation: " + operation);
  }

  if (!node[ParticipantDescriptionKey])
    throw YAML::ParserException(node.Mark(),
            "Expected a [" + ParticipantDescriptionKey + "] field");

  auto description = participant_description(node[ParticipantDescriptionKey]);

  return {op_type, description};
}

//==============================================================================
YAML::Node serialize(rmf_traffic_msgs::msg::ConvexShapeContext context)
{
  //For now since only circles are supported, so I'm just going to store their
  //Radii. In future this should change.
  YAML::Node node;
  for (auto circle : context.circles)
  {
    node.push_back(circle.radius);
  }
  return node;
}

//==============================================================================
std::string serialize_shape_type(uint8_t shape_type)
{
  if (shape_type == rmf_traffic_msgs::msg::ConvexShape::BOX)
    return BoxKey;
  if (shape_type == rmf_traffic_msgs::msg::ConvexShape::CIRCLE)
    return CircleKey;

  throw std::runtime_error("Shape type must be one of Box, Circle");
}

//==============================================================================
YAML::Node serialize(rmf_traffic_msgs::msg::ConvexShape shape)
{
  YAML::Node node;
  node[TypeKey] = serialize_shape_type(shape.type);
  node[IndexKey] = static_cast<uint16_t>(shape.index);
  return node;
}

//==============================================================================
YAML::Node serialize(rmf_traffic::Profile profile)
{
  YAML::Node node;
  rmf_traffic_msgs::msg::Profile profile_msg = convert(profile);
  node[FootprintKey] = serialize(profile_msg.footprint);
  node[VicinityKey] = serialize(profile_msg.vicinity);
  node[ShapeContextKey] = serialize(profile_msg.shape_context);
  return node;
}

//==============================================================================
std::string serialize_responsiveness(ParticipantDescription::Rx resp)
{
  if (resp == ParticipantDescription::Rx::Unresponsive)
    return UnresponsiveKey;
  if (resp == ParticipantDescription::Rx::Responsive)
    return ResponsiveKey;
  throw std::runtime_error("Failed to seriallize responsiveness");
}

//==============================================================================
YAML::Node serialize(ParticipantDescription participant)
{
  YAML::Node node;

  node[NameKey] = participant.name();
  node[GroupKey] = participant.owner();
  node[ResponsivenessKey] = serialize_responsiveness(
    participant.responsiveness());
  node[ProfileKey] = serialize(participant.profile());

  return node;
}

//==============================================================================
YAML::Node serialize(AtomicOperation atomOp)
{
  YAML::Node node;

  if (atomOp.operation == AtomicOperation::OpType::Add)
  {
    node[OperationKey] = AddOperationKey;
  }
  else
  {
    throw std::runtime_error("Found an invalid operation");
  }
  node[ParticipantDescriptionKey] = serialize(atomOp.description);

  return node;
}

} // namespace schedule
} // namespace rmf_traffic_ros2
