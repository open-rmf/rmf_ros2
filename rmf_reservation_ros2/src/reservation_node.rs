use std::env;

use anyhow::{Error, Result};
use rmf_chope_msgs::{srv::{ClaimReservation_Response, ClaimReservation_Request}, msg::FixedTimeReservationAlt};
use rmf_reservations::{database::{FixedTimeReservationSystem, Ticket}, ReservationParameters, ReservationRequest, cost_function::static_cost::StaticCost, StartTimeRange};

use std::sync::{Arc, Mutex};

struct ReservationService {
    res_sys: FixedTimeReservationSystem
}

fn reservation_req_service(service: Arc<Mutex<ReservationService>>) -> impl Fn(
    &rclrs::rmw_request_id_t,
    rmf_chope_msgs::srv::FixedTimeRequest_Request) -> rmf_chope_msgs::srv::FixedTimeRequest_Response {
    
    let service = service.clone();
    move |_req_id, requested_alternatives| {
        let mut service = service.lock().unwrap();
        let mut alternatives: Vec<_> = requested_alternatives.alternatives.iter().map(|requested_alternatives| {
            let start_time = chrono::DateTime::<chrono::Utc>::from_timestamp(requested_alternatives.start_time.sec.into(), requested_alternatives.start_time.nanosec.into());

            ReservationRequest {
                parameters: ReservationParameters {
                    resource_name: requested_alternatives.resource_name.clone(),
                    duration: if requested_alternatives.has_end {
                        Some(chrono::Duration::nanoseconds(requested_alternatives.duration.nanosec.into()) + chrono::Duration::seconds(requested_alternatives.duration.sec.into()))
                    } else {
                        None
                    },
                    start_time: StartTimeRange::exactly_at(&start_time.unwrap())
                },
                cost_function: Arc::new(StaticCost::new(requested_alternatives.cost))
            }
        }).collect();

        if alternatives.len() == 0 {
            return rmf_chope_msgs::srv::FixedTimeRequest_Response {
                ticket: 0,
                ok: false
            };
        }
        
        if let Ok(req) = service.res_sys.request_resources(alternatives) {
            rmf_chope_msgs::srv::FixedTimeRequest_Response {
                ticket: req.get_id() as u64,
                ok: true
            }
        }
        else {
            rmf_chope_msgs::srv::FixedTimeRequest_Response {
                ticket: 0,
                ok: false
            }
        }
    }
}


fn reservation_claim_service(service: Arc<Mutex<ReservationService>>) -> impl Fn(
    &rclrs::rmw_request_id_t,
    rmf_chope_msgs::srv::ClaimReservation_Request) -> rmf_chope_msgs::srv::ClaimReservation_Response {
    
    let service = service.clone();
    move |_req_id, claim| {
        let mut service = service.lock().unwrap();

        if let Some(result)= service.res_sys.claim_request(Ticket::from_id(claim.ticket as usize)) {
            rmf_chope_msgs::srv::ClaimReservation_Response {
                alternative: result as u64,
                ok: true
            }
        }
        else {
            rmf_chope_msgs::srv::ClaimReservation_Response {
                alternative: 0,
                ok: false
            }
        }

    }
}


fn main() -> Result<(), Error> {
    let context = rclrs::Context::new(env::args())?;

    let node = rclrs::create_node(&context, "rmf_reservation_service")?; 

    let res_sys = ReservationService {
        res_sys: FixedTimeReservationSystem::create_with_resources(
            vec!["pantry", "tinyRobot1_charger", "lounge", "supplies"].iter().map(|x| x.to_string()).collect()
        )
    };

    let res_sys = Arc::new(Mutex::new(res_sys));
    
    let reservation_req_cb = reservation_req_service(res_sys.clone());

    let claim_cb = reservation_claim_service(res_sys.clone());

    let _res_server = node
       .create_service::<rmf_chope_msgs::srv::FixedTimeRequest, _>("/rmf/reservation_node/request", reservation_req_cb)?;

    let _claim_server = node
       .create_service::<rmf_chope_msgs::srv::ClaimReservation, _>("/rmf/reservation_node/claim", claim_cb)?;

    println!("Starting server");
    rclrs::spin(node).map_err(|err| err.into())
}
