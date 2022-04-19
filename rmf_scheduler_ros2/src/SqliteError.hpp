/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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

#pragma once

#include <exception>
#include <memory>

class sqlite3;

namespace rmf::scheduler {

class SqliteError : public std::exception
{
public:
  int code;

  explicit SqliteError(sqlite3* db);
  explicit SqliteError(int code, const char* errmsg);
  const char* what() const noexcept override;

private: std::string errmsg;
};

}
