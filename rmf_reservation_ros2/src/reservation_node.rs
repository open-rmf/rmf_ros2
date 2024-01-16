use std::env;

use chrono::Utc;
use anyhow::{Error, Result};
use rmf_chope_msgs::msg::{FlexibleTimeRequest, DelayRequest, Ticket as RmfTicket, ClaimRequest, ReservationAllocation};
use rmf_reservations::{database::{FlexibleTimeReservationSystem, Ticket, ClockSource, }, ReservationParameters, ReservationRequest, cost_function::static_cost::StaticCost, StartTimeRange, };
use rmf_reservations::database::ClaimSpot;

use std::sync::{Arc, Mutex};
use std::collections::HashMap;

struct ReservationSystem {
    system: FlexibleTimeReservationSystem<RosClockSourceProvider>,
}

impl ReservationSystem {
    pub fn create_with_clock(clock_source: RosClockSourceProvider) -> Self {
        Self {
            system: FlexibleTimeReservationSystem::create_with_clock(clock_source)
        }
    }
}

struct RosClockSourceProvider {
    node: Arc<rclrs::Node>
}

impl ClockSource for RosClockSourceProvider {
    fn now(&self) -> chrono::DateTime<Utc> {
        return from_ros_time(&to_ros_msg(&self.node.get_clock().now()));
    }
}

fn to_ros_msg(time: &rclrs::Time) -> builtin_interfaces::msg::Time {
    let nanosec = time.nsec % 1_000_000_000;
    let sec = time.nsec /  1_000_000_000;

    builtin_interfaces::msg::Time {
        nanosec: nanosec.try_into().unwrap(),
        sec: sec.try_into().unwrap()
    }
}

fn to_ros_time(time: &chrono::DateTime<Utc>) -> builtin_interfaces::msg::Time {
    builtin_interfaces::msg::Time {
        nanosec: (time.timestamp_nanos() - time.timestamp() * 1_000_000_000).try_into().unwrap(),
        sec: time.timestamp().try_into().unwrap()
    }
}

fn from_ros_time(time: &builtin_interfaces::msg::Time) -> chrono::DateTime<Utc> {
    chrono::DateTime::<chrono::Utc>::from_timestamp(time.sec.into(), time.nanosec.into()).unwrap()
}

fn from_ros_duration(duration: &builtin_interfaces::msg::Duration) -> chrono::Duration {
    chrono::Duration::nanoseconds(duration.nanosec.into()) + chrono::Duration::seconds(duration.sec.into())
}

fn main() -> Result<(), Error> {
    let context = rclrs::Context::new(env::args())?;

    let node = rclrs::create_node(&context, "rmf_reservation_service")?;

    let clock_source= RosClockSourceProvider {
        node: node.clone()
    };
    let reservation_system = Arc::new(Mutex::new(ReservationSystem::create_with_clock(clock_source)));

    let res_sys_handle = reservation_system.clone();
    let publisher =
        node.create_publisher::<RmfTicket>("rmf/reservations/tickets", rclrs::QOS_PROFILE_DEFAULT)?;

    let _request_sub = node.create_subscription::<FlexibleTimeRequest, _>(
        "rmf/reservations/request",
        rclrs::QOS_PROFILE_DEFAULT,
        move |msg: FlexibleTimeRequest| {
            let alternative = msg.alternatives.iter().map(|alt| {
                ReservationRequest {
                    parameters: ReservationParameters {
                        resource_name: alt.resource_name.clone(),
                        duration: if alt.has_end {
                            Some(from_ros_duration(&alt.duration))
                        } else {
                            None
                        },
                        start_time: StartTimeRange {
                            earliest_start: if alt.start_time.has_earliest_start_time {
                                Some(from_ros_time(&alt.start_time.earliest_start_time))
                            } else {
                                None
                            },
                            latest_start: if alt.start_time.has_latest_start_time {
                                Some(from_ros_time(&alt.start_time.latest_start_time))
                            } else {
                                None
                            }
                        },
                    },
                    cost_function: Arc::new(StaticCost::new(alt.cost))
                }
            });

            println!("Recieved request.");

            //println!("I heard: '{}'", msg.data);
            let mut res_sys = res_sys_handle.lock().unwrap();
            let Ok(ticket) = res_sys.system.request_resources(alternative.collect()) else {
                println!("Could not claim ticket. It was likely already claimed");
                return;
            };
            //res_sys.mapping.insert(req_header, ticket.clone());
            let ticket = RmfTicket {
                header: msg.header,
                ticket_id: ticket.get_id() as u64
            };
            publisher.publish(ticket);
        },
    )?;

    let res_sys_handle = reservation_system.clone();
    let publisher =
        node.create_publisher::<ReservationAllocation>("rmf/reservations/allocation", rclrs::QOS_PROFILE_DEFAULT)?;
    let _claim_sub = node.create_subscription::<ClaimRequest, _>(
        "rmf/reservations/claim",
        rclrs::QOS_PROFILE_DEFAULT,
        move |msg: ClaimRequest| {
            let mut res_sys = res_sys_handle.lock().unwrap();
            let resp = res_sys.system.claim_request(Ticket::from_id(msg.ticket.ticket_id as usize), &msg.wait_points);
            let Ok(result) = resp  else {
                println!("Got error message while trying to claim: {}", resp.unwrap_err());
                return;
            };

            let allocation = match result {
                ClaimSpot::GoImmediately(goal) => {
                    ReservationAllocation {
                        ticket: msg.ticket.clone(),
                        instruction_type: ReservationAllocation::IMMEDIATELY_PROCEED,
                        satisfies_alternative: goal.satisfies_alt as u64,
                        resource: goal.resource.clone(),
                        time_to_reach: to_ros_time(&goal.time),
                        alternate_waitpoint: Default::default()
                    }
                },
                ClaimSpot::WaitAtThenGo(id, goal) => {
                    ReservationAllocation {
                        ticket: msg.ticket.clone(),
                        instruction_type: ReservationAllocation::WAIT_AT_SPOT_AND_THEN_GO,
                        satisfies_alternative: goal.satisfies_alt as u64,
                        resource: goal.resource.clone(),
                        time_to_reach: to_ros_time(&goal.time),
                        alternate_waitpoint: msg.wait_points[id].clone()
                    }
                }, 
                ClaimSpot::WaitPermanently(id) => {
                    ReservationAllocation {
                        ticket: msg.ticket.clone(),
                        instruction_type: ReservationAllocation::WAIT_PERMANENTLY,
                        satisfies_alternative: Default::default(),
                        resource: Default::default(),
                        time_to_reach: Default::default(),
                        alternate_waitpoint: msg.wait_points[id].clone()
                    }
                }
            };

            publisher.publish(allocation);
        }
    );    

    rclrs::spin(node).map_err(|err| err.into())
}
