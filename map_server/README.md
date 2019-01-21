# Usage

## Backward compatible usage

Run a map_saver node that subscribes to the /map topic, saves the map and exits.
```
rosrun map_server map_saver -f <map_name>
```

## Service based usage

Run a service_based_map_saver node that is an action server for map saving.
The action will subscribe to the /map topic,
write the available map to a file with the name given in the action goal,
and unsubscribe from the /map topic.
```
rosrun map_server service_based_map_saver
```
Call the action, e.g. by publishing to the goal topic:
```
rostopic pub /service_based_map_saver/goal move_base_msgs/SaveMapActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: '1'
goal:
  map_name: 'test'"
```
Once a map is available, the map will be saved. Trigger by writing a map to the /map topic:
```
rostopic pub /map nav_msgs/OccupancyGrid "header:
 seq: 0
 stamp:
   secs: 0
   nsecs: 0
 frame_id: ''
info:
 map_load_time: {secs: 0, nsecs: 0}
 resolution: 0.0
 width: 0
 height: 0
 origin:
   position: {x: 0.0, y: 0.0, z: 0.0}
   orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}
data: [0]"
```
