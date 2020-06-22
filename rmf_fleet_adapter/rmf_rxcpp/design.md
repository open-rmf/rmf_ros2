
This is a temporary document for drafting the initial reactive design of the
fleet adapters. This file should be replaced with proper documentation after
the first round of implementation is finished.

In the main event loop of the fleet adapter, we will need to have a ROS2 node
performing spin_some(). The subscriptions of the node should trigger
higher-order observables to produce results that internal rxcpp subscribers can
receive. For example we will want to have higher-order rxcpp observables for the
following ROS2 topic subscriptions (to name a few):

* `delivery_requests`
* `door_states`
* `lift_states`

## Delivery request

When a delivery request comes in over the `delivery_requests` topic, it should
trigger the fleet adapter to begin a planning job observable. When the planning
job is complete, it should trigger a subscriber that will create a chain of
observable+subscriber pairs. Each subscriber in the chain will spawn the next
observable+subscriber pair. For now we would need four types of
observable+subscriber pairs:

* Robot path following (subscriber gets triggered when the robot reaches the end
of its path)
* Door opening (subscriber gets triggered when the door is done opening)
* Door closing (subscriber gets triggered when the door is done closing)
* Lift summoning (subscriber gets triggered when the lift arrives at the desired
floor and its doors are open)

## Robot path following

The plan result subscriber will have access to a
[`std::shared_ptr<RobotCommandHandle>`](https://github.com/osrf/rmf_mock_adapter/blob/42fbab49f7f9fbac0e4cb00cabfa83dd81f3108e/rmf_mock_adapter/include/rmf_mock_adapter/adapter.hpp#L43)
which allows it to issue a robot a path following command. It will also issue
a callback for the fleet adapter to trigger when the robot has reached the end
of the path. When the callback is triggered, it will issue out an observable
result whose subscriber will spawn the next observable+subscriber pair. That
new pair may be to open a door, summon a lift, request a dispenser, etc.

## Doors

Interactions with doors will require a chain of subscriptions and observables.
When the robot needs to pass through a door, it will approach the door after
being given a robot path following command. When the robot path following
command is finished, its completion callback will be triggered. The completion
callback will then spawn a door state subscriber with these characteristics:

* If the door mode is not open, periodically publish a command for the door to open
* If the door mode is open, trigger the next "robot path following" command to move the robot through the door

The "robot path following" command that moves the robot through the door will
give the fleet adapter a completion callback that creates a new door state
subscriber with these properties:

* If the door mode is not closed, periodically publish a command for the door to close
* If the door mode is closed, trigger the next "robot path following" command to have the robot proceed to its destination

## Lifts

The chain of observables+subscribers for lifts is similar to doors, except that
they watch `lift_states` and has four parts:

1. Summon the lift to the robot's current floor and wait for it to arrive
2. Move the robot into the lift
3. Command the lift to go to the robot's destination floor and wait for it to arrive
4. Move the robot out of the lift
