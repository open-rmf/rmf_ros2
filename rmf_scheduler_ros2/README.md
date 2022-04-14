# rmf\_scheduler\_ros2 package

Manages scheduled events for RMF. This is the server component of `rmf_schedule`, for the client components, see [rmf_scheduler_ros2_client](../rmf_scheduler_ros2_client/README.md).

## Topics
| Topic | Message | Description |
| --- | --- | --- |
| new_trigger | rmf_scheduler_msgs/Trigger | Published when a new trigger is created or replaced. |
| new_schedule | rmf_scheduler_msgs/Schedule | Published when a new schedule is created or replaced. |
| trigger_update | rmf_scheduler_msgs/TriggerState | Published when a trigger is ran. |
| schedule_update | rmf_scheduler_msgs/ScheduleState | Published when a schedule is ran. |

## Services
| Service | Type | Description |
| --- | --- | --- |
| create_trigger | rmf_scheduler_msgs/CreateTrigger | Create a new trigger or replaces an existing trigger. |
| create_schedule | rmf_scheduler_msgs/CreateSchedule | Create a new schedule or replaces an existing schedule. |
| cancel_trigger | rmf_scheduler_msgs/CancelTrigger | Cancel an existing trigger. |
| cancel_schedule | rmf_scheduler_msgs/CancelSchedule | Cancel an existing schedule. |
| list_triggers | rmf_scheduler_msgs/ListTriggers | List triggers in the system. |
| list_trigger_states | rmf_scheduler_msgs/ListTriggerStates | List trigger states in the system. |
| list_schedules | rmf_scheduler_msgs/ListSchedules | List schedules in the system. |
| list_schedule_states | rmf_scheduler_msgs/ListScheduleStates | List schedule states in the system. |

## Quality Declaration

This package claims to be in the **Quality Level 4** category. See the [Quality Declaration](QUALITY_DECLARATION.md) for more details.
