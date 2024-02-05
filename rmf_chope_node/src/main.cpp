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


const std::string ReservationRequestTopicName = "/rmf/reservations/request";
const std::string ReservationResponseTopicName = "/rmf/reservations/tickets";
const std::string ReservationClaimTopicName = "/rmf/reservations/claim";
const std::string ReservationAllocationTopicName = "/rmf/reservations/allocation";
const std::string ReservationReleaseTopicName = "/rmf/reservations/release";


/// C++-isms
template <>
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
struct RobotIdentifier {
  std::string robot_name;
  std::string fleet_name;

  bool operator==(const RobotIdentifier& other) const {
    return robot_name == other.robot_name && fleet_name == other.fleet_name;
  }
};


template <>
struct std::hash<RobotIdentifier>
{
  std::size_t operator()(const RobotIdentifier& header) const
  {
    using std::size_t;
    using std::hash;
    using std::string;


    return ((hash<string>()(header.robot_name)
             ^ (hash<string>()(header.fleet_name) << 1)) >> 1);
  }
};


/// Ticket generation class for book keeping purposes. Will eventually overflow and leak memory.
/// Ticket id 0 does not exist and is useful for making emergency claims.
class TicketStore {

public:
  rmf_chope_msgs::msg::Ticket get_new_ticket(const rmf_chope_msgs::msg::RequestHeader &request_header)
  {
    rmf_chope_msgs::msg::Ticket ticket;
    ticket.header = request_header;
    ticket.ticket_id =  _last_issued_ticket_id;
    RobotIdentifier robot_identifier {
      request_header.robot_name,
      request_header.fleet_name
    };
    _robots_to_tickets[robot_identifier].push_back(_last_issued_ticket_id);   
    _header_to_ticket.emplace(request_header, _last_issued_ticket_id);
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

  std::unordered_map<RobotIdentifier, std::vector<std::size_t>> _robots_to_tickets;
  std::unordered_map<rmf_chope_msgs::msg::RequestHeader, std::size_t> _header_to_ticket;
  std::unordered_map<std::size_t, rmf_chope_msgs::msg::RequestHeader> _ticket_to_header;
  std::size_t _last_issued_ticket_id = 1;
};

struct LocationState {
  std::optional<std::size_t> ticket;
};

struct LocationReq {
  std::string location;
  double cost;

  bool operator<(const LocationReq& other) const  {
    return cost < other.cost;
  }

  bool operator==(const LocationReq& other) const  {
    return cost == other.cost && location == other.location;
  }
};

/// Implements a simple Mutex. Only one robot can claim a location at a time.
/// The current implementation is relatively simplistic and basically checks if a location is occupied or not.
/// A queuing system is in the works. Note: It is possible for the current system to get deadlocked.
class CurrentState {
public:  
  std::vector<std::string> free_locations() {
    std::lock_guard<std::mutex> lock(_mtx);
    std::vector<std::string> locations;
    for (auto &[loc, state] : _current_location_reservations) {
      if (!state.ticket.has_value()) {
        locations.push_back(loc);
      }
    }
    return locations;
  }

  void add_location(std::string location) {
    std::lock_guard<std::mutex> lock(_mtx);
    if (_current_location_reservations.count(location) == 0)
    {
      _current_location_reservations.emplace(location, LocationState {std::nullopt});
    }
  }
  
  std::optional<std::string> allocate_lowest_cost_free_spot(const std::vector<LocationReq>& incoming_requests, const std::size_t ticket_id)
  {
    if (_ticket_to_location.count(ticket_id) != 0)
    {
      // Ticket has been allocated. Probably some DDS-ism causing the issue
      return std::nullopt;
    }

    auto requests = incoming_requests;
    std::sort(requests.begin(), requests.end());
    std::lock_guard<std::mutex> lock(_mtx);
    for (std::size_t i = 0; i < requests.size(); i++) {
      auto parking = _current_location_reservations.find(requests[i].location);
      if (parking == _current_location_reservations.end()) {
        _current_location_reservations[requests[i].location] = LocationState {ticket_id};
        _ticket_to_location.emplace(ticket_id, requests[i].location);
        return requests[i].location;
      }
      else if (!parking->second.ticket.has_value()) {
        _current_location_reservations[requests[i].location].ticket = ticket_id;
        _ticket_to_location.emplace(ticket_id, requests[i].location);
        return parking->first;
      }
    }

    return std::nullopt;
  }

  bool release(const std::size_t ticket_id)
  {
    std::lock_guard<std::mutex> lock(_mtx);
    auto _ticket = _ticket_to_location.find(ticket_id);
    if (_ticket == _ticket_to_location.end())
    {
      return false;
    }
    auto location = _ticket->second;
    _current_location_reservations[location].ticket = std::nullopt;
    return true;
  }

private:
  std::mutex _mtx;
  std::unordered_map<std::string, LocationState> _current_location_reservations;
  std::unordered_map<std::size_t, std::string> _ticket_to_location;
};

using namespace std::chrono_literals;

class SimpleQueueSystemNode : public rclcpp::Node {
public:
  SimpleQueueSystemNode()  : Node("rmf_chope_node") {

    rclcpp::QoS qos(10);
    qos = qos.reliable();
    qos = qos.keep_last(10);
    qos = qos.transient_local();

    request_subscription_ = this->create_subscription<rmf_chope_msgs::msg::FlexibleTimeRequest>(
      ReservationRequestTopicName, qos, std::bind(&SimpleQueueSystemNode::on_request, this, std::placeholders::_1));
    claim_subscription_ = this->create_subscription<rmf_chope_msgs::msg::ClaimRequest>(
      ReservationClaimTopicName, qos, std::bind(&SimpleQueueSystemNode::claim_request, this, std::placeholders::_1));
    release_subscription_ = this->create_subscription<rmf_chope_msgs::msg::ReleaseRequest>(
      ReservationReleaseTopicName, qos, std::bind(&SimpleQueueSystemNode::release, this, std::placeholders::_1));
    graph_subscription_ = this->create_subscription<rmf_building_map_msgs::msg::Graph>("/nav_graphs", qos, std::bind(&SimpleQueueSystemNode::recieved_graph, this, std::placeholders::_1));

    ticket_pub_ = this->create_publisher<rmf_chope_msgs::msg::Ticket>(ReservationResponseTopicName, qos);
    allocation_pub_ = this->create_publisher<rmf_chope_msgs::msg::ReservationAllocation>(ReservationAllocationTopicName, qos);
    free_spot_pub_ = this->create_publisher<rmf_chope_msgs::msg::FreeParkingSpots>("/rmf/reservations/free_parking_spot", qos);

    timer_ = this->create_wall_timer(500ms, std::bind(&SimpleQueueSystemNode::publish_free_spots, this));
  }

private:
  void recieved_graph(const rmf_building_map_msgs::msg::Graph::ConstSharedPtr &graph_msg) {
    RCLCPP_INFO(this->get_logger(), "Got graph");
    for (std::size_t i = 0; i < graph_msg->vertices.size(); i++)
    {
      for(auto &param: graph_msg->vertices[i].params) {
        
        //TODO(arjo) make this configure-able
        if (param.name == "is_parking_spot" && param.value_bool)
        {
          std::stringstream name;
          name << i;
          std::string topic;
          name >> topic;
          current_state_.add_location(topic);
        }
      }
    }
  }
  void on_request(const rmf_chope_msgs::msg::FlexibleTimeRequest::ConstSharedPtr &request) {
    
    std::vector<LocationReq> requests;

    for (auto alt: request->alternatives) {
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

  void claim_request(const rmf_chope_msgs::msg::ClaimRequest::ConstSharedPtr &request) {
    
    // This logic is for the simplified queue-less version.
    std::vector<LocationReq> locations;
    
    for (auto claim: requests_[request->ticket.ticket_id])
    {
      locations.push_back(claim);
    }
    
    auto cost = (locations.size() == 0) ? 0.0: locations.back().cost + 1.0;
    for (auto claim: request->wait_points) {
      locations.push_back(LocationReq {
        claim,
        cost
      });
      cost += 1.0;
    }
    auto result = current_state_.allocate_lowest_cost_free_spot(locations, request->ticket.ticket_id);
    if (result.has_value())
    {
      rmf_chope_msgs::msg::ReservationAllocation allocation;
      allocation.ticket  = ticket_store_.get_existing_ticket(request->ticket.ticket_id);
      allocation.instruction_type = rmf_chope_msgs::msg::ReservationAllocation::IMMEDIATELY_PROCEED;
      
      allocation.resource = result.value();

      for (std::size_t i=0; i < requests_[request->ticket.ticket_id].size(); i++)
      {
        if (requests_[request->ticket.ticket_id][i].location  == result.value())
        {
          allocation.satisfies_alternative = i;          
        }
      }
      RCLCPP_INFO(this->get_logger(), "Allocating %s to %lu", result.value().c_str(), request->ticket.ticket_id);
      allocation_pub_->publish(allocation);
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Could not allocate resource for ticket %lu.", request->ticket.ticket_id);
    }
  }

  void release(const rmf_chope_msgs::msg::ReleaseRequest::ConstSharedPtr &request) {
    RCLCPP_INFO(this->get_logger(), "Releasing ticket for %lu", request->ticket.ticket_id);
    auto ticket = request->ticket.ticket_id;
    auto success = current_state_.release(ticket);
    if (!success) {
      RCLCPP_ERROR(this->get_logger(), "Could not find ticker %lu", request->ticket.ticket_id);
    }
  }


  void publish_free_spots() {
    rmf_chope_msgs::msg::FreeParkingSpots spots;
    spots.spots = current_state_.free_locations();

    free_spot_pub_->publish(spots);
  }

  rclcpp::Subscription<rmf_chope_msgs::msg::FlexibleTimeRequest>::SharedPtr request_subscription_;
  rclcpp::Subscription<rmf_chope_msgs::msg::ClaimRequest>::SharedPtr claim_subscription_;
  rclcpp::Subscription<rmf_chope_msgs::msg::ReleaseRequest>::SharedPtr release_subscription_;
  rclcpp::Subscription<rmf_building_map_msgs::msg::Graph>::SharedPtr graph_subscription_;

  rclcpp::Publisher<rmf_chope_msgs::msg::Ticket>::SharedPtr ticket_pub_;
  rclcpp::Publisher<rmf_chope_msgs::msg::ReservationAllocation>::SharedPtr allocation_pub_;
  rclcpp::Publisher<rmf_chope_msgs::msg::FreeParkingSpots>::SharedPtr free_spot_pub_;

  std::unordered_map<std::size_t, std::vector<LocationReq>> requests_;
  TicketStore ticket_store_;
  CurrentState current_state_;

  rclcpp::TimerBase::SharedPtr timer_;
};

int main (int argc, const char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleQueueSystemNode>());
  rclcpp::shutdown();
  return 0;
}