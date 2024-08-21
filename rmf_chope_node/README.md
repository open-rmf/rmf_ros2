# rmf\_chope\_node package

"Chope" is the very Singaporean act of throwing an item on a table and marking it at yours before going to get your food. The goal of this package is to provide the same functionality in RMF.
Before a robot goes to its next destination the fleet adapter asks the chope-node if its next destination is available. If the destination is available the robot will immediately proceed to the next destination, otherwise it will allocate a free parking spot for the robot to wait at till the next destination becomes available in a FIFO manner. If you need more advanced methods than FIFO, feel free to hack this package. To enable the use of this package you need to add the following to your fleet configuration yaml at the fleet level and run the chope node:
```yaml
  use_parking_reservations: True
```
We recommend disabling `responsive_wait` when you do so. An example is available in `rmf_demos`.

Some immediate limitations of the chope node is that it is limited to scenarios with a single navigation graph. Overcoming this limitation should not be too much work now that the basic infrastructure is in place.

## Expected Behaviour

### Basic queueing
You should never need to interact with this package directly. Rather, you can simply dispatch a task using the traditional methods. If you have `rmf_demos` installed you can run the office world. We can start by commanding the `tinyRobot2` to go to the pantry.
```bash
ros2 run rmf_demos_tasks dispatch_go_to_place -p pantry -F tinyRobot -R tinyRobot2 --use_sim_time
```
The robot should proceed as expected. We can then ask `tinyRobot1` to also go to the pantry. Nothing should happen as the pantry is already occupied and `tinyRobot1` is at its parking spot.
```bash
ros2 run rmf_demos_tasks dispatch_go_to_place -p pantry  -F tinyRobot -R tinyRobot1 --use_sim_time
```
We can ask `tinyRobot2` to move to the lounge after this.
```bash
ros2 run rmf_demos_tasks dispatch_go_to_place -p lounge -F tinyRobot -R tinyRobot2 --use_sim_time
```
This should immediately trigger 2 things:
1. `tinyRobot2` will move to the lounge
2. `tinyRobot1` will proceed to move to the pantry taking tinyRobot2's place.

### Moving to a waitpoint
If we continue from the previous example. Lets ask `tinyRobot1` and `tinyRobot2` to swap places starting at the pantry and lounge respectively.
```bash
ros2 run rmf_demos_tasks dispatch_go_to_place -p pantry -F tinyRobot -R tinyRobot1 --use_sim_time
```
You should see tinyRobot1 move to`tinyRobot2_charger` as `lounge` is not a designated wait point. You can then command `tinyRobot2` to the lounge.
```bash
ros2 run rmf_demos_tasks dispatch_go_to_place -p lounge -F tinyRobot -R tinyRobot2 --use_sim_time
```
`tinyRobot1` and `tinyRobot2` should now proceed to swap places.

If you have the same number or more waitpoints than robots you should never have a deadlock.

## Protocol Used Behind the Scenes

The chope node has no information about the state of the graph, it simply maintains a list of available parking spots internally. The fleet adapter does most of the heavy lifting in the `GoToPlace` event. When a `GoToPlace` event is started we first check if the robot is already at one of the target locations. The fleet adapter submits a ReservationRequest message with the desired end parameters. The parameters are listed in terms of cost (where the cost function is distance). The chope node will issue a ticket for said request. When ready to proceed, send a claim message with the ticket and list of potential waiting points ordered by distance. The chope node will then try to award the lowest cost target location. If it can't it will award the lowest cost waiting point to the robot. The fleet adapter will release the previous location as it starts to move to its next location.


## Known Issues
1. At start up if there is no idle task, the chope node will not know where the robots are. It is advised to send 1 `GoToPlace` task for every robot that is added to the world.

## Quality Declaration

This package claims to be in the **Quality Level 4** category. See the [Quality Declaration](QUALITY_DECLARATION.md) for more details._