# rmf\_scheduler\_ros2\_client package

Client library for rmf\_scheduler\_ros2.

## Usage

### Creating a Trigger

```cpp
#include <rclcpp/rclcpp.hpp>

#include <rmf_scheduler_client/payload.hpp>

#include <rmf_scheduler_msgs/srv/create_trigger.hpp>
#include <std_msgs/msg/string.hpp>

#include <chrono>
#include <iostream>

using namespace std::chrono_literals;

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("example_node");
  auto client = node->create_client<rmf_scheduler_msgs::srv::CreateTrigger>(
    "create_trigger");
  client->wait_for_service(5s);

  std_msgs::msg::String msg;
  msg.data = "hello world";

  auto req =
    std::make_shared<rmf_scheduler_msgs::srv::CreateTrigger::Request>();
  req->trigger.name = "my_trigger";
  req->trigger.at = 100;
  req->trigger.payload = rmf::scheduler::make_serialized_message("test_topic",
      msg);

  auto resp_fut = client->async_send_request(req);
  rclcpp::spin_until_future_complete(node, resp_fut);
  if (resp_fut.get()->success)
  {
    std::cout << "success" << std::endl;
  }
}
```

### Creating a Schedule

Schedules are described using a cron string.

```
┌─────────────seconds (0 - 59)
│ ┌───────────── minute (0 - 59)
│ │ ┌───────────── hour (0 - 23)
│ │ │ ┌───────────── day of the month (1 - 31)
│ │ │ │ ┌───────────── month (1 - 12)
│ │ │ │ │ ┌───────────── day of the week (0 - 6) (Sunday to Saturday)
│ │ │ │ │ │ ┌───────────── years (1970 - 2099) (optional)
│ │ │ │ │ │ │
* * * * * * *
```

| Field | Required | Allowed value | Allowed special characters |
| --- | --- | --- | --- |
| seconds | yes | 0-59 | `*` `,` `-` |
| minutes | yes | 0-59 | `*` `,` `-` |
| hours | yes | 0-23 | `*` `,` `-` |
| days of month | 1-31 | `*` `,` `-` `?` `L` `W` |
| months | yes | 1-12 | `*` `,` `-` |
| days of week | yes | `*` `,` `-` `?` `L` `#` |
| years | no | 1970-2099 | `*` `,` `-` |

The special characters have the following meaning:

| Special character | Meaning | Description |
| --- | --- | --- |
| `*` | all values | selects all values within a field |
| `?` | no specific value | specify one field and leave the other unspecified |
| `-` | range | specify ranges |
| `,` | comma | specify additional values |
| `/` | slash | speficy increments |
| `L` | last | last day of the month or last day of the week |
| `W` | weekday | the weekday nearest to the given day |
| `#` | nth |  specify the Nth day of the month |
Examples: 

| CRON | Description |
| --- | --- |
| * * * * * * | Every second |
| */5 * * * * ? | Every 5 seconds |
| 0 */5 */2 * * ? | Every 5 minutes, every 2 hours |
| 0 */2 */2 ? */2 */2 | Every 2 minutes, every 2 hours, every 2 days of the week, every 2 months |
| 0 15 10 * * ? * | 10:15 AM every day |
| 0 0/5 14 * * ? | Every 5 minutes starting at 2 PM and ending at 2:55 PM, every day |
| 0 10,44 14 ? 3 WED | 2:10 PM and at 2:44 PM every Wednesday of March |
| 0 15 10 ? * MON-FRI | 10:15 AM every Monday, Tuesday, Wednesday, Thursday and Friday |
| 0 15 10 L * ? | 10:15 AM on the last day of every month |
| 0 0 12 1/5 * ? | 12 PM every 5 days every month, starting on the first day of the month |
| 0 11 11 11 11 ? | Every November 11th at 11:11 AM |

Reference: https://github.com/mariusbancila/croncpp/blob/999f7685ab683b58872386c0aa019acf97c6570a/README.md

```cpp
#include <rclcpp/rclcpp.hpp>

#include <rmf_scheduler_client/payload.hpp>

#include <rmf_scheduler_msgs/srv/create_schedule.hpp>
#include <std_msgs/msg/string.hpp>

#include <chrono>
#include <iostream>

using namespace std::chrono_literals;

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("example_node");
  auto client = node->create_client<rmf_scheduler_msgs::srv::CreateSchedule>(
    "create_schedule");
  client->wait_for_service(5s);

  std_msgs::msg::String msg;
  msg.data = "hello world";

  auto req =
    std::make_shared<rmf_scheduler_msgs::srv::CreateSchedule::Request>();
  req->schedule.name = "my_schedule";
  req->schedule.schedule = "* * * * * *";
  req->schedule.payload = rmf::scheduler::make_serialized_message("test_topic",
      msg);

  auto resp_fut = client->async_send_request(req);
  rclcpp::spin_until_future_complete(node, resp_fut);
  if (resp_fut.get()->success)
  {
    std::cout << "success" << std::endl;
  }
}
```

## Other Endpoints

A list of topics and services exposed by the scheduler node can be found [here](../rmf_scheduler_ros2/README.md). They can use used just like any other ros2 endpoints.

## Quality Declaration

This package claims to be in the **Quality Level 4** category. See the [Quality Declaration](QUALITY_DECLARATION.md) for more details.
