/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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


#ifndef SRC__RMF_TRAFFIC_ROS2__GEOMETRY__SHAPEINTERNAL_HPP
#define SRC__RMF_TRAFFIC_ROS2__GEOMETRY__SHAPEINTERNAL_HPP

#include <functional>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <vector>

namespace rmf_traffic_ros2 {
namespace geometry {
namespace internal {

//==============================================================================
template<typename ShapeType, typename ShapeMsgType, typename ContextMsgType>
class ShapeContextImpl
{
public:

  using Base = ShapeContextImpl<ShapeType, ShapeMsgType, ContextMsgType>;
  using ShapeTypePtr = std::shared_ptr<const ShapeType>;

  std::vector<std::vector<ShapeTypePtr>> shapes;

  struct Entry
  {
    std::size_t type;
    std::size_t index;
  };

  using EntryMap = std::unordered_map<ShapeTypePtr, Entry>;
  EntryMap entry_map;

  using Caster = std::function<std::size_t(const ShapeTypePtr&)>;
  static std::mutex initialization_mutex;
  static bool initialized;
  static std::vector<Caster> casters;
  static std::size_t num_shape_types;

  ShapeContextImpl() {}

  template<typename DerivedShape>
  void add(const std::size_t type_index)
  {
    casters.push_back(
      [=](const ShapeTypePtr& shape) -> std::size_t
      {
        if (dynamic_cast<const DerivedShape*>(&shape->source()))
          return type_index;

        return 0;
      });

    if (type_index >= num_shape_types)
    {
      num_shape_types = type_index+1;
      shapes.resize(num_shape_types);
    }
  }

  std::size_t get_type_index(const ShapeTypePtr& shape)
  {
    for (const auto& caster : casters)
    {
      const std::size_t cast = caster(shape);
      if (cast != 0)
        return cast;
    }

    // *INDENT-OFF*
    throw std::runtime_error(
      std::string()
      + "Failed to find a shape context type index for type ["
      + typeid(shape->source()).name() + "] with base type ["
      + typeid(ShapeType).name() + "]");
    // *INDENT-ON*
  }

  ShapeMsgType insert(ShapeTypePtr shape)
  {
    if (!shape)
    {
      ShapeMsgType shape_msg;
      shape_msg.type = 0;
      shape_msg.index = 0;
      return shape_msg;
    }

    const auto insertion =
      entry_map.insert(std::make_pair(shape, Entry{}));

    const bool inserted = insertion.second;
    Entry& entry = insertion.first->second;
    if (inserted)
    {
      entry.type = get_type_index(shape);

      std::vector<ShapeTypePtr>& derived_shapes = shapes.at(entry.type);
      entry.index = derived_shapes.size();
      derived_shapes.push_back(shape);
    }

    ShapeMsgType shape_msg;
    shape_msg.type = entry.type;
    shape_msg.index = entry.index;

    return shape_msg;
  }

  ShapeTypePtr at(const ShapeMsgType& shape) const
  {
    if (shape.type == ShapeMsgType::NONE)
      return nullptr;

    const std::vector<ShapeTypePtr>& bucket = shapes.at(shape.type);
    return bucket.at(shape.index);
  }

};

//==============================================================================
template<class T, class M, class C>
std::mutex ShapeContextImpl<T, M, C>::initialization_mutex;

//==============================================================================
template<class T, class M, class C>
bool ShapeContextImpl<T, M, C>::initialized = false;

//==============================================================================
template<class T, class M, class C>
std::size_t ShapeContextImpl<T, M, C>::num_shape_types = 0;

//==============================================================================
template<class T, class M, class C>
std::vector<typename ShapeContextImpl<T, M, C>::Caster>
ShapeContextImpl<T, M, C>::casters;

} // namespace internal
} // namespace geometry
} // namespace rmf_traffic_ros2

#endif // SRC__RMF_TRAFFIC_ROS2__GEOMETRY__SHAPEINTERNAL_HPP
