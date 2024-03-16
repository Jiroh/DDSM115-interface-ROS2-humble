# ros2_micro-ROS_DDSM115_m5StampS3
DDSM115 interface ROS2 micro-ROS on m5StampS3


## Dependency
- ROS2 Humble
- micro_ros_arduino-humble

## Prepare Hardware
### Build a your own bot


## Getting started
Edit keys_example.h for your access point 

char wifi_ssid[20] = "WIFI_SSID";
char wifi_pass[20] = "WIFI_PASS";
char agent_ip[20] = "192.168.128.111";
unsigned int agent_port = 8888;
 rename as "keys.h"```

## Build and Upload
```
please use VS code
```

## Run ROS2 nodes for sending commands
### Run micro-ROS agent
```
$ ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
``````
### Run teleop_node
```
$ ros2 launch teleop_twist_joy teleop-launch.py joy_config:='xbox'
```

### Useful commands for debugging
```
$ ros2 topic list
$ ros2 topic echo /joy
$ ros2 topic echo /cmd_vel

