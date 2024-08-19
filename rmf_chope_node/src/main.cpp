#include "rmf_chope_msgs/msg/flexible_time_request.hpp"
#include "rmf_chope_msgs/msg/claim_request.hpp"
#include "rclcpp/rclcpp.hpp"
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <optional>
#include <rclcpp/logging.hpp>

#include <rmf_building_map_msgs/msg/graph.hpp>
#include <rmf_chope_msgs/msg/detail/free_parking_spots__struct.hpp>
#include <rmf_chope_msgs/msg/request_header.hpp>
#include <rmf_chope_msgs/msg/release_request.hpp>
#include <rmf_chope_msgs/msg/reservation_allocation.hpp>
#include <rmf_chope_msgs/msg/ticket.hpp>
#include <rmf_chope_msgs/msg/free_parking_spots.hpp>
#include <unordered_map>
#include <vector>

#include <iostream>

const std::string ReservationRequestTopicName = "/rmf/reservations/request";
const std::string ReservationResponseTopicName = "/rmf/reservations/tickets";
const std::string ReservationClaimTopicName = "/rmf/reservations/claim";
const std::string ReservationAllocationTopicName =
  "/rmf/reservations/allocation";
const std::string ReservationReleaseTopicName = "/rmf/reservations/release";


/// C++-isms
template<>
struct std::hash<rmf_chope_msgs::msg::RequestHeader>
{
  std::size_t operator()(const rmf_chope_msgs::msg::RequestHeader& header) const
  {
    using std::size_t;
    using std::hash;
    using std::string;


    return ((hash<string>()(header.robot_name)
      ^ (hash<string>()(header.fleet_name) << 1)) >> 1)
      ^ (hash<uint64_t>()(header.request_id) << 1);
  }
};


/// Useful for identifying which tickets belong to which robots.
struct RobotIdentifier
{
  std::string robot_name;
  std::string fleet_name;

  bool operator==(const RobotIdentifier& other) const
  {
    return robot_name == other.robot_name && fleet_name == other.fleet_name;
  }
};


template<>
struct std::hash<RobotIdentifier>
{
  std::size_t operator()(const RobotIdentifier& header) const
  {
    using std::size_t;
    using std::hash;
    using std::string;


    return (hash<string>()(header.robot_name)
      ^ (hash<string>()(header.fleet_name) << 1)) >> 1;
  }
};


/// Ticket generation class for book keeping purposes. Will eventually overflow.
/// Ticket id 0 does not exist and is useful for making emergency claims.
/// Ticket ids are mapped across multiple fleets.
class TicketStore
{

public:
  rmf_chope_msgs::msg::Ticket get_new_ticket(
    const rmf_chope_msgs::msg::RequestHeader& request_header)
  {
    rmf_chope_msgs::msg::Ticket ticket;
    ticket.header = request_header;
    ticket.ticket_id = _last_issued_ticket_id;
    RobotIdentifier robot_identifier {
      request_header.robot_name,
      request_header.fleet_name
    };
    _ticket_to_header.emplace(_last_issued_ticket_id, request_header);
    _last_issued_ticket_id++;
    return ticket;
  }

  rmf_chope_msgs::msg::Ticket get_existing_ticket(const std::size_t index)
  {
    rmf_chope_msgs::msg::Ticket ticket;
    ticket.ticket_id = index;
    ticket.header = _ticket_to_header[index];
    return ticket;
  }

  std::unordered_map<std::size_t,
    rmf_chope_msgs::msg::RequestHeader> _ticket_to_header;
  std::size_t _last_issued_ticket_id = 1;
};

struct LocationState
{
  std::optional<std::size_t> ticket;
};

struct LocationReq
{
  std::string location;
  double cost;

  bool operator<(const LocationReq& other) const
  {
    return cost < other.cost;
  }

  bool operator==(const LocationReq& other) const
  {
    return cost == other.cost && location == other.location;
  }
};


/// Implements a simple Mutex. Only one robot can claim a location at a time.
/// The current implementation is relatively simplistic and basically checks
/// if a location is occupied or not. A queuing system is in the works.
class CurrentState
{
public:
  std::vector<std::string> free_locations()
  {
    std::vector<std::string> locations;
    for (auto&[loc, state] : _current_location_reservations)
    {
      if (!state.ticket.has_value())
      {
        locations.push_back(loc);
      }
    }
    return locations;
  }

  void add_location(std::string location)
  {
    if (_current_location_reservations.count(location) == 0)
    {
      std::cerr << "Got duplicate location [" << location
        << "]" << std::endl;
      _current_location_reservations.emplace(location,
        LocationState {std::nullopt});
    }
  }

  /// Tries to greedily allocate the lowest cost free spot given a list of potential
  /// parking spots.
  /// \param[in] incoming_requests - Parking spot and cost of said parking spot.
  /// \param[in] ticket_id - Ticket which is being serviced.
  std::optional<std::size_t> allocate_lowest_cost_free_spot(
    const std::vector<LocationReq>& incoming_requests,
    const std::size_t ticket_id)
  {
    if (_ticket_to_location.count(ticket_id) != 0)
    {
      /// Release previous instance and reallocate ticket
      release(ticket_id);
    }

    std::unordered_map<std::string, std::size_t> positions;
    for (std::size_t i = 0; i < incoming_requests.size(); ++i)
    {
      positions[incoming_requests[i].location] = i;
    }

    auto requests = incoming_requests;
    std::sort(requests.begin(), requests.end());
    for (std::size_t i = 0; i < requests.size(); i++)
    {
      auto parking = _current_location_reservations.find(requests[i].location);
      if (parking == _current_location_reservations.end())
      {
        // New parking spot not in list. Should be fine to occupy.
        _current_location_reservations[requests[i].location] =
          LocationState {ticket_id};
        _ticket_to_location.emplace(ticket_id, requests[i].location);
        return positions[requests[i].location];
      }
      else if (!parking->second.ticket.has_value())
      {
        // Existing parking spot.
        _current_location_reservations[requests[i].location].ticket = ticket_id;
        _ticket_to_location.emplace(ticket_id, requests[i].location);
        return positions[parking->first];
      }
    }

    std::cerr << "Could not free space from any of: ";
    for (auto c: requests) {
      std::cerr << c.location << ", ";
    }
    std::cerr << "\n";

    return std::nullopt;
  }

  std::optional<std::string> release(const std::size_t ticket_id)
  {
    auto _ticket = _ticket_to_location.find(ticket_id);
    if (_ticket == _ticket_to_location.end())
    {
      return std::nullopt;
    }
    auto location = _ticket->second;
    _current_location_reservations[location].ticket = std::nullopt;
_ticket_to_location.erase(_ticket);
    return {location};
  }

private:
  std::unordered_map<std::string, LocationState> _current_location_reservations;
  std::unordered_map<std::size_t, std::string> _ticket_to_location;
};


///A queue that allows removal of an item based on its value.
template<typename T>
class ItemQueue
{
public:
  /// Adds an item to the queue.
  /// Time complexity: O(1)
  void add(T item)
  {
    index_to_item[curr_index] = item;
    item_to_index[item] = curr_index;
    indices.insert(curr_index);
    curr_index++;
  }

  // Removes the item from the queue
  // Log(n)
  void remove_item(T item)
  {
    auto index = item_to_index.find(item);
    if (index == item_to_index.end())
    {
      return;
    }
    item_to_index.erase(item);
    index_to_item.erase(index->second);
    indices.erase(index->second);
  }

  // Gives the most recent item in the queue.
  // Returns nullopt if the queue is empty.
  std::optional<T> front()
  {
    auto it = indices.begin();
    if (it == indices.end())
    {
      return std::nullopt;
    }
    return index_to_item[*it];
  }

  std::size_t curr_index = 0;

  std::unordered_map<std::size_t, T> index_to_item;
  std::unordered_map<T, std::size_t> item_to_index;
  std::set<std::size_t> indices;
};

/// This class enqueues items based on how old a request is.
/// The basic idea is that we maintain a queue for every resource. As requests
/// come in we simultaneously add them to every queue which can be serviced.
/// Once a resource becomes free we call `service_next_in_queue` for said resource.
/// When we service the next item in the queue we remove it from all other queues.
class ServiceQueueManager
{
  std::unordered_map<std::string,
    ItemQueue<std::size_t>> resource_queues;
public:
  std::optional<std::size_t> service_next_in_queue(const std::string& resource)
  {
    auto item = resource_queues[resource].front();
    if (!item.has_value())
    {
      return std::nullopt;
    }

    for (auto& [_, resource_queue]: resource_queues)
    {
      resource_queue.remove_item(item.value());
    }
    return item;
  }

  void add_to_queue(std::size_t ticket, std::vector<std::string>& resources)
  {
    for(auto resource: resources)
    {
      resource_queues[resource].add(ticket);
    }
  }
};

using namespace std::chrono_literals;

class ChopeNode : public rclcpp::Node
{
public:
  ChopeNode()
  : Node("rmf_chope_node")
  {

    rclcpp::QoS qos(10);
    qos = qos.reliable();
    qos = qos.keep_last(10);
    qos = qos.transient_local();

    request_subscription_ =
      this->create_subscription<rmf_chope_msgs::msg::FlexibleTimeRequest>(
      ReservationRequestTopicName, qos,
      std::bind(&ChopeNode::on_request, this,
      std::placeholders::_1));
    claim_subscription_ =
      this->create_subscription<rmf_chope_msgs::msg::ClaimRequest>(
      ReservationClaimTopicName, qos,
      std::bind(&ChopeNode::claim_request, this,
      std::placeholders::_1));
    release_subscription_ =
      this->create_subscription<rmf_chope_msgs::msg::ReleaseRequest>(
      ReservationReleaseTopicName, qos,
      std::bind(&ChopeNode::release, this, std::placeholders::_1));
    graph_subscription_ =
      this->create_subscription<rmf_building_map_msgs::msg::Graph>(
      "/nav_graphs", qos,
      std::bind(&ChopeNode::recieved_graph, this,
      std::placeholders::_1));

    ticket_pub_ = this->create_publisher<rmf_chope_msgs::msg::Ticket>(
      ReservationResponseTopicName, qos);
    allocation_pub_ =
      this->create_publisher<rmf_chope_msgs::msg::ReservationAllocation>(
      ReservationAllocationTopicName, qos);
    free_spot_pub_ =
      this->create_publisher<rmf_chope_msgs::msg::FreeParkingSpots>(
      "/rmf/reservations/free_parking_spot", qos);

    timer_ =
      this->create_wall_timer(500ms,
        std::bind(&ChopeNode::publish_free_spots, this));
  }

private:
  void recieved_graph(
    const rmf_building_map_msgs::msg::Graph::ConstSharedPtr& graph_msg)
  {
    RCLCPP_INFO(this->get_logger(), "Got graph");
    for (std::size_t i = 0; i < graph_msg->vertices.size(); i++)
    {
      for (auto& param: graph_msg->vertices[i].params)
      {

        //TODO(arjoc) make this configure-able
        if (param.name == "is_parking_spot" && param.value_bool)
        {
          current_state_.add_location( graph_msg->vertices[i].name);
        }
      }
    }
  }
  void on_request(
    const rmf_chope_msgs::msg::FlexibleTimeRequest::ConstSharedPtr& request)
  {

    std::vector<LocationReq> requests;

    for (auto alt: request->alternatives)
    {
      LocationReq request{
        alt.resource_name,
        alt.cost
      };

      requests.push_back(request);
    }

    auto ticket = ticket_store_.get_new_ticket(request->header);
    requests_[ticket.ticket_id] = requests;
    ticket_pub_->publish(ticket);
  }

  void claim_request(
    const rmf_chope_msgs::msg::ClaimRequest::ConstSharedPtr& request)
  {

    // This logic is for the simplified queue-less version.
    std::vector<LocationReq> locations;
    std::vector<std::string> location_names;

    for (auto location_pref: requests_[request->ticket.ticket_id])
    {
      locations.push_back(location_pref);
      location_names.push_back(location_pref.location);
    }

    // Allocate the lowest cost free spot from list of intended final locations if possible
    auto result = current_state_.allocate_lowest_cost_free_spot(locations,
        request->ticket.ticket_id);
    if (result.has_value())
    {
      rmf_chope_msgs::msg::ReservationAllocation allocation;
      allocation.ticket = ticket_store_.get_existing_ticket(
        request->ticket.ticket_id);
      allocation.instruction_type =
        rmf_chope_msgs::msg::ReservationAllocation::IMMEDIATELY_PROCEED;
      allocation.satisfies_alternative = result.value();
      allocation.resource = requests_[request->ticket.ticket_id][result.value()].location;

      RCLCPP_INFO(this->get_logger(), "Allocating %s to %lu",
        allocation.resource.c_str(), request->ticket.ticket_id);
      allocation_pub_->publish(allocation);
      return;
    }

    // If we can't proceed immediately add the ticket to a queue.
    RCLCPP_INFO(this->get_logger(), "Could not immediately service %lu, enqueing.",
      request->ticket.ticket_id);
    queue_manager_.add_to_queue(request->ticket.ticket_id, location_names);

    // Allocate a waitpoint by preference as given by Fleet Adapter
    std::vector<LocationReq> wait_points;
    auto cost = 0.0;
    for (auto waitpoint_name: request->wait_points)
    {

      LocationReq request{
        waitpoint_name,
        cost
      };
      wait_points.push_back(request);
      cost += 1.0;
    }

    waitpoints_[request->ticket.ticket_id] = wait_points;
    auto waitpoint_result = current_state_.allocate_lowest_cost_free_spot(wait_points,
        request->ticket.ticket_id);
    if (waitpoint_result.has_value())
    {
      rmf_chope_msgs::msg::ReservationAllocation allocation;
      allocation.ticket = ticket_store_.get_existing_ticket(
        request->ticket.ticket_id);
      allocation.instruction_type =
        rmf_chope_msgs::msg::ReservationAllocation::WAIT_PERMANENTLY;
      allocation.satisfies_alternative = waitpoint_result.value();
      allocation.resource = wait_points[waitpoint_result.value()].location;
      RCLCPP_INFO(this->get_logger(), "Allocating %s as waitpoint to %lu",
                allocation.resource.c_str(), request->ticket.ticket_id);
      allocation_pub_->publish(allocation);
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Could not allocate a waiting point for robots from %lu waitpoints", wait_points.size());
    }
  }

  void release(
    const rmf_chope_msgs::msg::ReleaseRequest::ConstSharedPtr& request)
  {
    RCLCPP_INFO(
      this->get_logger(), "Releasing ticket for %lu",
      request->ticket.ticket_id);
    auto ticket = request->ticket.ticket_id;
    auto released_location = current_state_.release(ticket);
    if (!released_location.has_value())
    {
      RCLCPP_ERROR(
        this->get_logger(), "Could not find ticket %lu",
        request->ticket.ticket_id);
      return;
    }

    auto next_item = queue_manager_.service_next_in_queue(released_location.value());
    if (!next_item.has_value())
    {
      RCLCPP_INFO(
        this->get_logger(), "Queue is now empty %s",
        released_location->c_str());
      return;
    }
    // Will go to lowest
    auto result = current_state_.allocate_lowest_cost_free_spot(requests_[next_item.value()],
          next_item.value());
    RCLCPP_INFO(
        this->get_logger(), "Found next item %lu on queue %s",
        next_item.value(),
        released_location.value().c_str());

    if (!result.has_value())
    {
      RCLCPP_ERROR(
        this->get_logger(), "Tried to service %lu. Apparently there was some inconsitency between the chope node's state and the", ticket);
      return;
    }
    rmf_chope_msgs::msg::ReservationAllocation allocation;
    allocation.satisfies_alternative = result.value();
    allocation.resource = requests_[next_item.value()][result.value()].location;
    allocation.ticket = ticket_store_.get_existing_ticket(next_item.value());
    allocation.instruction_type =
      rmf_chope_msgs::msg::ReservationAllocation::IMMEDIATELY_PROCEED;
    RCLCPP_INFO(this->get_logger(), "Allocating %s to %lu",
      allocation.resource.c_str(), next_item.value());
    allocation_pub_->publish(allocation);
  }


  void publish_free_spots()
  {
    rmf_chope_msgs::msg::FreeParkingSpots spots;
    spots.spots = current_state_.free_locations();

    free_spot_pub_->publish(spots);
  }

  rclcpp::Subscription<rmf_chope_msgs::msg::FlexibleTimeRequest>::SharedPtr
    request_subscription_;
  rclcpp::Subscription<rmf_chope_msgs::msg::ClaimRequest>::SharedPtr
    claim_subscription_;
  rclcpp::Subscription<rmf_chope_msgs::msg::ReleaseRequest>::SharedPtr
    release_subscription_;
  rclcpp::Subscription<rmf_building_map_msgs::msg::Graph>::SharedPtr
    graph_subscription_;

  rclcpp::Publisher<rmf_chope_msgs::msg::Ticket>::SharedPtr ticket_pub_;
  rclcpp::Publisher<rmf_chope_msgs::msg::ReservationAllocation>::SharedPtr
    allocation_pub_;
  rclcpp::Publisher<rmf_chope_msgs::msg::FreeParkingSpots>::SharedPtr
    free_spot_pub_;

  std::unordered_map<std::size_t, std::vector<LocationReq>> requests_;
  std::unordered_map<std::size_t, std::vector<LocationReq>> waitpoints_;
  TicketStore ticket_store_;
  CurrentState current_state_;
  ServiceQueueManager queue_manager_;

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, const char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ChopeNode>());
  rclcpp::shutdown();
  return 0;
}