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

#include "NegotiationRoom.hpp"

#include <rmf_traffic_ros2/Route.hpp>
#include <rmf_traffic_ros2/schedule/Itinerary.hpp>

#include <iostream>

namespace rmf_traffic_ros2 {

//==============================================================================
rmf_traffic::schedule::Negotiation::VersionedKeySequence convert(
  const std::vector<rmf_traffic_msgs::msg::NegotiationKey>& from)
{
  rmf_traffic::schedule::Negotiation::VersionedKeySequence output;
  output.reserve(from.size());
  for (const auto& key : from)
    output.push_back({key.participant, key.version});

  return output;
}

//==============================================================================
std::vector<rmf_traffic_msgs::msg::NegotiationKey> convert(
  const rmf_traffic::schedule::Negotiation::VersionedKeySequence& from)
{
  std::vector<rmf_traffic_msgs::msg::NegotiationKey> output;
  output.reserve(from.size());
  for (const auto& key : from)
  {
    rmf_traffic_msgs::msg::NegotiationKey msg;
    msg.participant = key.participant;
    msg.version = key.version;
    output.push_back(msg);
  }

  return output;
}

namespace schedule {

//==============================================================================
NegotiationRoom::NegotiationRoom(rmf_traffic::schedule::Negotiation negotiation_)
: negotiation(std::move(negotiation_))
{
  // Do nothing
}

//==============================================================================
std::vector<rmf_traffic::schedule::Negotiation::TablePtr> NegotiationRoom::
check_cache(const NegotiatorMap& negotiators)
{
  std::vector<rmf_traffic::schedule::Negotiation::TablePtr> new_tables;

  bool recheck = false;
  do
  {
    recheck = false;

    for (auto it = cached_proposals.begin(); it != cached_proposals.end(); )
    {
      const auto& proposal = *it;
      const auto search = negotiation.find(
        proposal.for_participant, convert(proposal.to_accommodate));

      if (search.deprecated())
      {
        cached_proposals.erase(it++);
        continue;
      }

      const auto table = search.table;
      if (table)
      {
        const bool updated = table->submit(
          proposal.plan_id,
          rmf_traffic_ros2::convert(proposal.itinerary),
          proposal.proposal_version);

        if (updated)
          new_tables.push_back(table);

        recheck = true;
        cached_proposals.erase(it++);
      }
      else
        ++it;
    }

    for (auto it = cached_rejections.begin(); it != cached_rejections.end(); )
    {
      const auto& rejection = *it;
      const auto search = negotiation.find(convert(rejection.table));

      if (search.deprecated())
      {
        cached_rejections.erase(it++);
        continue;
      }

      const auto table = search.table;
      if (table)
      {
        table->reject(
          rejection.table.back().version,
          rejection.rejected_by,
          rmf_traffic_ros2::convert(rejection.alternatives));
        recheck = true;
        cached_rejections.erase(it++);
      }
      else
        ++it;
    }

    for (auto it = cached_forfeits.begin(); it != cached_forfeits.end(); )
    {
      const auto& forfeit = *it;
      const auto search = negotiation.find(convert(forfeit.table));

      if (search.deprecated())
      {
        cached_forfeits.erase(it++);
        continue;
      }

      const auto table = search.table;
      if (table)
      {
        table->forfeit(forfeit.table.back().version);
        recheck = true;
        cached_forfeits.erase(it++);
      }
      else
        ++it;
    }

  } while (recheck);

  auto remove_it = std::remove_if(new_tables.begin(), new_tables.end(),
      [](const rmf_traffic::schedule::Negotiation::TablePtr& table)
      {
        return table->forfeited() || table->defunct();
      });
  new_tables.erase(remove_it, new_tables.end());

  std::vector<rmf_traffic::schedule::Negotiation::TablePtr> respond_to;
  for (const auto& new_table : new_tables)
  {
    for (const auto& n : negotiators)
    {
      const auto participant = n.first;
      if (const auto r = new_table->respond(participant))
        respond_to.push_back(r);
    }
  }

  return respond_to;
}

//==============================================================================
void print_negotiation_status(
  rmf_traffic::schedule::Version conflict_version,
  const rmf_traffic::schedule::Negotiation& negotiation)
{
  using Negotiation = rmf_traffic::schedule::Negotiation;

  std::cout << "\n[" << conflict_version << "] Active negotiation:";
  for (const auto p : negotiation.participants())
    std::cout << " " << p;
  std::cout << std::endl;

  std::vector<Negotiation::ConstTablePtr> terminal;
  std::vector<Negotiation::ConstTablePtr> queue;
  for (const auto p : negotiation.participants())
  {
    const auto p_table = negotiation.table(p, {});
    assert(p_table);
    queue.push_back(p_table);
  }

  while (!queue.empty())
  {
    Negotiation::ConstTablePtr top = queue.back();
    queue.pop_back();

    std::vector<Negotiation::ConstTablePtr> children = top->children();
    if (children.empty())
    {
      terminal.push_back(top);
    }
    else
    {
      for (const auto& child : children)
      {
        assert(child);
        queue.push_back(child);
      }
    }
  }

  std::cout << "    Current negotiation state";
  for (const auto& t : terminal)
  {
    std::cout << "\n --";
    const bool finished = static_cast<bool>(t->submission());
    const bool rejected = t->rejected();
    const bool forfeited = t->forfeited();
    const auto sequence = t->sequence();
    for (std::size_t i = 0; i < sequence.size(); ++i)
    {
      const auto& s = sequence[i];
      if (i == t->sequence().size()-1)
      {
        if (forfeited)
          std::cout << " <" << s.participant << ":" << s.version << ">";
        else if (rejected)
          std::cout << " {" << s.participant << ":" << s.version << "}";
        else if (!finished)
          std::cout << " [" << s.participant << ":" << s.version << "]";
        else
          std::cout << " >" << s.participant << ":" << s.version << "<";
      }
      else
      {
        std::cout << " " << s.participant << ":" << s.version;
      }
    }
  }
  std::cout << "\n" << std::endl;
}

} // namespace schedule
} // namespace rmf_traffic_ros2
