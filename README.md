# robotx_ekf
package for extended kalman filter
```
ros2 run robotx_ekf robotx_ekf_node
```
## have to
- determin dt carefully
- set proper variance
- it has filtering step twice (1. y=CX 2.gps observation)

## connect to VRX gazebo
- Firstly, run wamvtan/vrx
```bash
docker run -it wamvtan/vrx /bin/bash
ros2 launch vrx_gz competition.launch.py world:=stationkeeping_task headless:=true urdf:=/home/config/wamv_target.urdf
```
- Next, run another docker_image wamvtan/dev_container which includes our autonomous system.
```bash
docker run -it wamvtan/dev_container /bin/bash
ros2 run vrx_bridge vrx_bridge_node
ros2 launch vrx_gz competition.launch.py world:=stationkeeping_task headless:=true urdf:=/home/config/wamv_target.urdf
```
- if you want to run vrx_gz on you local computer, you don't need to run container.
- And do this command below.you run robotx_ekf_node and geopose_converter and vrx_bridge
```bash
ros2 run vrx_bridge vrx_bridge_node
ros2 launch vrx_gz competition.launch.py world:=stationkeeping_task headless:=true urdf:=/home/config/wamv_target.urdf
```
- you can see `/estimated_pose` topic
