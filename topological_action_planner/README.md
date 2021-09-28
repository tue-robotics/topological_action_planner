# topological_action_planner
ROS package for planning over a topological navigation mesh. 
Nodes are areas of interest in an ED world model and edges between these areas are labeled with the action to take to traverse the edge,
    like opening a door or plain driving.

## Try it out
Run 
```bash
roslaunch topological_action_planner start.launch
rosservice call /hero/topological_action_planner/get_plan  "origin:
  entity: 'operator_table'
  area: 'in_front_of'
destination:
  entity: 'dinner_table'
  area: 'in_front_of'"
```
or 
```bash
rosservice call /hero/topological_action_planner/get_plan  "origin:
  entity: ''  # Empty means robot's current location
  area: ''
destination:
  entity: 'dinner_table'
  area: 'in_front_of'"
``````
## Assumptions
### Robot can drive to all nodes in the same room
This might not be true, objects can block the path of course. 
This needs to be detected and then handled via a PUSH_OBJECT action or fail

## Optimisations:
### Collapse drive actions into one
Users/clients of this planner's service might optimize the plans further.
For example, if a plan to go from one area to another is linked by just DRIVE actions,
    those might be collapsed into just driving straight to the destination area.
It makes no sense to traverse a room just via some waypoints in that room instead of taking the shortest path.

### Navigation within a room
Consider 'inside' area of a room as a node and connect that to every node within the room?