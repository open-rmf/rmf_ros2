#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <rmf_traffic_msgs/msg/reservation.hpp>
#include <rmf_traffic_msgs/msg/reservation_requests.hpp>
#include <rmf_traffic_msgs/msg/reservation_rollout.hpp>
#include <rmf_traffic_msgs/msg/reservation_register_participant.hpp>
#include <rmf_traffic_msgs/msg/reservation_participant_heart_beat.hpp>
#include <rmf_traffic_msgs/msg/reservation_proposal.hpp>
#include <rmf_traffic_msgs/msg/reservation_proposal_ack.hpp>
#include <rmf_traffic_msgs/msg/reservation_proposal_rej.hpp>
#include <rmf_traffic_msgs/msg/reservation_rollout.hpp>
#include <rmf_traffic_msgs/msg/reservation_cancel_rollout.hpp>

#include <rmf_traffic_ros2/StandardNames.hpp>

namespace rmf_traffic_ros2 {

class ReservationManager : public rclcpp::Node
{
  public:
    ReservationManager()
      : Node("reservation_manager")
    {
      using std::placeholders::_1;

      rclcpp::QoS qos(10);

      // Publishers
      _reservation_proposal_pub =
        create_publisher<rmf_traffic_msgs::msg::ReservationProposal>(
          ReservationProposalTopicName, qos.reliable());
      _reservation_rollout_pub =
        create_publisher<rmf_traffic_msgs::msg::ReservationRollout>(
          ReservationRolloutTopicName, qos.reliable());

      _reservation_requests_sub =
        create_subscription<rmf_traffic_msgs::msg::ReservationRequests>(
          ReservationRequestTopicName,
          qos,
          std::bind(&ReservationManager::on_request_reservation, this, _1)
        );

      _reservation_proposal_ack_sub =
        create_subscription<rmf_traffic_msgs::msg::ReservationProposalAck>(
          ReservationProposalAckTopicName,
          qos,
          std::bind(&ReservationManager::on_proposal_acceptance, this, _1)
        );

      _reservation_proposal_rej_sub =
        create_subscription<rmf_traffic_msgs::msg::ReservationProposalRej>(
          ReservationProposalAckTopicName,
          qos,
          std::bind(&ReservationManager::on_proposal_rejection, this, _1)
        );

      _reservation_participant_registration_sub =
        create_subscription<rmf_traffic_msgs::msg::ReservationRegisterParticipant>(
          ReservationProposalAckTopicName,
          qos,
          std::bind(&ReservationManager::on_participant_register, this, _1)
        );

      _reservation_participant_heartbeat_sub =
        create_subscription<rmf_traffic_msgs::msg::ReservationParticipantHeartBeat>(
          ReservationUpdateHeartbeatTopicName,
          qos,
          std::bind(&ReservationManager::on_participant_heartbeat, this, _1)
        );
    }


  private:
    void on_request_reservation(
      const rmf_traffic_msgs::msg::ReservationRequests::SharedPtr req)
    {

    }

    void on_proposal_acceptance(
      const rmf_traffic_msgs::msg::ReservationProposalAck::SharedPtr participant)
    {

    }

    void on_proposal_rejection(
      const rmf_traffic_msgs::msg::ReservationProposalRej::SharedPtr participant)
    {

    }

    void on_participant_register(
      const rmf_traffic_msgs::msg::ReservationRegisterParticipant::SharedPtr participant)
    {

    }

    void on_participant_heartbeat(
      const rmf_traffic_msgs::msg::ReservationParticipantHeartBeat::SharedPtr participant)
    {

    }

    rclcpp::TimerBase::SharedPtr timer_;

    // Publishers
    rclcpp::Publisher<rmf_traffic_msgs::msg::ReservationProposal>::SharedPtr
      _reservation_proposal_pub;
    rclcpp::Publisher<rmf_traffic_msgs::msg::ReservationRollout>::SharedPtr
      _reservation_rollout_pub;

    //Subscribers
    rclcpp::Subscription<rmf_traffic_msgs::msg::ReservationRequests>::SharedPtr
      _reservation_requests_sub;
    rclcpp::Subscription<rmf_traffic_msgs::msg::ReservationProposalAck>::SharedPtr
      _reservation_proposal_ack_sub;
    rclcpp::Subscription<rmf_traffic_msgs::msg::ReservationProposalRej>::SharedPtr
      _reservation_proposal_rej_sub;
    rclcpp::Subscription<rmf_traffic_msgs::msg::ReservationRegisterParticipant>::SharedPtr
      _reservation_participant_registration_sub;
    rclcpp::Subscription<rmf_traffic_msgs::msg::ReservationParticipantHeartBeat>::SharedPtr
      _reservation_participant_heartbeat_sub;
};

}