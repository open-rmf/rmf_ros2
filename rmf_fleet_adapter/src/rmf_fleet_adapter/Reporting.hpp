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

#ifndef SRC__RMF_FLEET_ADAPTER__REPORTING_HPP
#define SRC__RMF_FLEET_ADAPTER__REPORTING_HPP

#include <rmf_task/Log.hpp>
#include <nlohmann/json.hpp>

#include <rxcpp/rx-includes.hpp>

#include <memory>
#include <unordered_set>

namespace rmf_fleet_adapter {

//==============================================================================
class Reporting
{
public:

  struct Issue
  {
    std::string category;
    nlohmann::json detail;
  };

  using IssuePtr = std::shared_ptr<const Issue>;
  using OpenIssues = std::unordered_set<IssuePtr>;

  struct Upstream : public std::enable_shared_from_this<Upstream>
  {
    Upstream(rxcpp::schedulers::worker worker_);

    OpenIssues open_issues;
    rmf_task::Log log;
    rxcpp::schedulers::worker worker;
    std::mutex mutex;
  };

  class Ticket
  {
  public:

    void resolve(nlohmann::json msg);

    // TODO(MXG): Should it be possible to change the category/detail of the
    // issue from the ticket?

    ~Ticket();

    friend class Reporting;
  private:
    Ticket(IssuePtr issue, std::shared_ptr<Upstream> upstream);

    IssuePtr _issue;
    std::weak_ptr<Upstream> _upstream;
  };

  Reporting(rxcpp::schedulers::worker worker);

  std::mutex& mutex() const;

  std::unique_ptr<Ticket> create_issue(
    rmf_task::Log::Tier tier,
    std::string category,
    nlohmann::json detail);

  const std::unordered_set<IssuePtr>& open_issues() const;

  rmf_task::Log& log();

  const rmf_task::Log& log() const;

private:
  std::shared_ptr<Upstream> _data;
};

} // namespace rmf_fleet_adapter

#endif // SRC__RMF_FLEET_ADAPTER__REPORTING_HPP
