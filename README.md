# How to run

``` bash
ros2 run ground_station command_node --ros-args \
    -p drone_domains:="[1,2]" -p drone_namespaces:="['drone1','drone2']"
```