# robotx_ekf
package for extended kalman filter
```
ros2 run robotx_ekf robotx_ekf_node
```
## have to
- determin dt carefully
- set proper variance
- it has filtering step twice (1. y=CX 2.gps observation)
