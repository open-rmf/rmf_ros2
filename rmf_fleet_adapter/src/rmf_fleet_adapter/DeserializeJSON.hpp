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

#ifndef SRC__RMF_FLEET_ADAPTER__DESERIALIZEJSON_HPP
#define SRC__RMF_FLEET_ADAPTER__DESERIALIZEJSON_HPP

#include <unordered_map>
#include <nlohmann/json.hpp>
#include <nlohmann/json-schema.hpp>

namespace rmf_fleet_adapter {

//==============================================================================
template<typename Deserialized>
class DeserializeJSON
{
public:

  using Deserializer = std::function<Deserialized(const nlohmann::json&)>;
  using Validator = nlohmann::json_schema::json_validator;

  struct Handlers
  {
    std::shared_ptr<const Validator> validator;
    Deserializer deserializer;
  };

  void add(
    const std::string& category,
    Validator validator,
    Deserializer deserializer)
  {
    handlers.insert_or_assign(
      category,
      Handlers{
        std::make_shared<Validator>(std::move(validator)),
        std::move(deserializer)
      });
  }

  void add(
    const std::string& category,
    std::shared_ptr<const Validator> validator,
    Deserializer deserializer)
  {
    handlers.insert_or_assign(
      category,
      Handlers{std::move(validator), std::move(deserializer)});
  }

  std::unordered_map<std::string, Handlers> handlers;

};

template<typename T>
using DeserializeJSONPtr = std::shared_ptr<DeserializeJSON<T>>;

} // namespace rmf_fleet_adapter

#endif // SRC__RMF_FLEET_ADAPTER__DESERIALIZEJSON_HPP
