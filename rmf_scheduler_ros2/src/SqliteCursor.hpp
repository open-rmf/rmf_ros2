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

#include "SqliteError.hpp"

#include <sqlite3.h>

#include <limits>
#include <functional>
#include <optional>
#include <stdexcept>
#include <string>

namespace rmf::scheduler {

template<typename T>
class SqliteCursor
{
public:
  // using std::function here is less efficient than template but simplifies
  // creating of cursors greatly.
  using Factory = std::function<T(sqlite3_stmt*)>;

  class Iter
  {
  private:
    std::shared_ptr<sqlite3> _db;
    sqlite3_stmt* _stmt;
    size_t _n = 0;
    Factory _f;

  public:
    Iter(sqlite3* db, sqlite3_stmt* stmt, size_t n, Factory f)
    : _db(db), _stmt(stmt), _n(n), _f(f)
    {
    }

    bool operator!=(const Iter& other) const
    {
      return this->_n != other._n || this->_stmt != other._stmt;
    }

    Iter& operator++()
    {
      int result = sqlite3_step(this->_stmt);
      if (result == SQLITE_DONE)
      {
        // turn this into past-the-end iterator
        this->_n = std::numeric_limits<size_t>::max();
        return *this;
      }
      if (result != SQLITE_ROW)
      {
        throw SqliteError{this->_db};
      }
      ++this->_n;
      return *this;
    }

    T operator*() const
    {
      return this->_f(this->_stmt);
    }
  };

public:
  std::shared_ptr<sqlite3> _db;
  sqlite3_stmt* _stmt;
  Iter _begin;
  Iter _end;

  /// Takes ownership of `stmt`.
  SqliteCursor(std::shared_ptr<sqlite3> db, sqlite3_stmt* stmt, Factory f)
  : _db(db),
    _stmt(stmt),
    _begin{stmt, 0, f},
    _end{stmt, std::numeric_limits<size_t>::max(), f}
  {
    ++this->_begin;
  }

  // disallow copying and moving to simplify lifetime management of sqlite3_stmt
  SqliteCursor(const SqliteCursor&) = delete;
  SqliteCursor(SqliteCursor&&) = delete;
  SqliteCursor& operator=(const SqliteCursor&) = delete;
  SqliteCursor& operator=(SqliteCursor&&) = delete;

  ~SqliteCursor()
  {
    if (sqlite3_finalize(this->_stmt) != SQLITE_OK)
    {
      throw SqliteError{this->_db};
    }
  }

  Iter& begin()
  {
    return this->_begin;
  }

  Iter& end()
  {
    return this->_end;
  }
};

}
