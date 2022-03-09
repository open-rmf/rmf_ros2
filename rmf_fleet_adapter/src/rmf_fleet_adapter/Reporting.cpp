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

#include "Reporting.hpp"

namespace rmf_fleet_adapter {

//==============================================================================
Reporting::Upstream::Upstream(rxcpp::schedulers::worker worker_)
: worker(std::move(worker_))
{
  // Do nothing
}

//==============================================================================
void Reporting::Ticket::resolve(nlohmann::json msg)
{
  const auto upstream = _upstream.lock();
  if (!upstream)
    return;

  if (!_issue)
    return;

  upstream->worker.schedule(
    [
      w = upstream->weak_from_this(),
      msg = std::move(msg),
      issue = std::move(_issue)
    ](const auto&)
    {
      const auto upstream = w.lock();
      if (!upstream)
        return;

      std::lock_guard<std::mutex> lock(upstream->mutex);
      if (upstream->open_issues.erase(issue) > 0)
      {
        upstream->log.info(
          "Resolved issue [" + issue->category + "]: " + msg.dump());
      }
    });

  _issue = nullptr;
}

//==============================================================================
Reporting::Ticket::~Ticket()
{
  const auto upstream = _upstream.lock();
  if (!upstream)
    return;

  if (!_issue)
    return;

  upstream->worker.schedule(
    [
      w = upstream->weak_from_this(),
      issue = std::move(_issue)
    ](const auto&)
    {
      const auto upstream = w.lock();
      if (!upstream)
        return;

      std::lock_guard<std::mutex> lock(upstream->mutex);

      if (upstream->open_issues.erase(issue) > 0)
        upstream->log.warn("Dropped issue [" + issue->category + "]");
    });
}

//==============================================================================
Reporting::Ticket::Ticket(IssuePtr issue, std::shared_ptr<Upstream> upstream)
: _issue(std::move(issue)),
  _upstream(std::move(upstream))
{
  // Do nothing
}

//==============================================================================
Reporting::Reporting(rxcpp::schedulers::worker worker_)
: _data(std::make_shared<Upstream>(std::move(worker_)))
{
  // Do nothing
}

//==============================================================================
std::mutex& Reporting::mutex() const
{
  return _data->mutex;
}

//==============================================================================
std::unique_ptr<Reporting::Ticket> Reporting::create_issue(
  rmf_task::Log::Tier tier,
  std::string category,
  nlohmann::json detail)
{
  auto issue = std::make_shared<Issue>(
    Issue{std::move(category), std::move(detail)});

  std::lock_guard<std::mutex> lock(_data->mutex);
  _data->log.push(
    tier, "Opened issue [" + issue->category + "]: " + issue->detail.dump());

  _data->open_issues.insert(issue);
  return std::unique_ptr<Ticket>(new Ticket(issue, _data));
}

//==============================================================================
auto Reporting::open_issues() const -> const std::unordered_set<IssuePtr>&
{
  return _data->open_issues;
}

//==============================================================================
rmf_task::Log& Reporting::log()
{
  return _data->log;
}

//==============================================================================
const rmf_task::Log& Reporting::log() const
{
  return _data->log;
}

} // namespace rmf_fleet_adapter
