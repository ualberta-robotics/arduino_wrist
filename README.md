## WAM4DOF+WRIST-WAM7DOF
### REQUIREMENTS
### LINKING ARMS
```
rosservice call /zeus/wam/link_arm 192.168.1.11
rosservice call /slax/bhand/link_arm 192.168.1.10
```
### Connecting wrist
```
rosrun rosserial_python serial_node.py /dev/ttyACM0
rosrun encoder_to_wam_wrist encoder_to_wrist
```

### RUN
