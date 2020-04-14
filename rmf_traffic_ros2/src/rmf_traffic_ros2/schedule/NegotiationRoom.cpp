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

#include <iostream>

namespace rmf_traffic_ros2 {
namespace schedule {

//==============================================================================
NegotiationRoom::NegotiationRoom(rmf_traffic::schedule::Negotiation negotiation_)
  : negotiation(std::move(negotiation_))
{
  // Do nothing
}

//==============================================================================
std::vector<rmf_traffic::schedule::Negotiation::TablePtr> NegotiationRoom::check_cache()
{
  std::vector<rmf_traffic::schedule::Negotiation::TablePtr> new_tables;

  bool recheck = false;
  do
  {
    recheck = false;

    for (auto it = cached_proposals.begin(); it != cached_proposals.end();)
    {
      const auto& proposal = *it;
      const auto table = negotiation.table(
            proposal.for_participant, proposal.to_accommodate);
      if (table)
      {
        const bool updated = table->submit(
              rmf_traffic_ros2::convert(
                proposal.itinerary), proposal.proposal_version);
        if (updated)
          new_tables.push_back(table);

        recheck = true;
        cached_proposals.erase(it++);
      }
      else
        ++it;
    }

    for (auto it = cached_rejections.begin(); it != cached_rejections.end();)
    {
      // TODO(MXG): This needs to account for what version of the proposal
      // is being rejected.
      const auto& rejection = *it;
      const auto table = negotiation.table(rejection.table);
      if (table)
      {
        table->reject(rejection.proposal_version);
        recheck = true;
        cached_rejections.erase(it++);
      }
      else
        ++it;
    }

  } while (recheck);

  auto remove_it = std::remove_if(new_tables.begin(), new_tables.end(),
                 [](const rmf_traffic::schedule::Negotiation::TablePtr& table)
  {
    return table->rejected();
  });
  new_tables.erase(remove_it, new_tables.end());

  return new_tables;
}

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
    const auto sequence = t->sequence();
    for (std::size_t i=0; i < sequence.size(); ++i)
    {
      if (i == t->sequence().size()-1 && rejected)
        std::cout << " <" << sequence[i] << ">";
      else if (i == t->sequence().size()-1 && !finished)
        std::cout << " [" << sequence[i] << "]";
      else
        std::cout << " " << sequence[i];
    }
  }
  std::cout << "\n" << std::endl;
}

} // namespace schedule
} // namespace rmf_traffic_ros2
