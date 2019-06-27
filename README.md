## WAM4DOF (SLAX) + WRIST LINKED WITH WAM7DOF (ZEUS)
### Connecting to Zeus and Slax 
Run from the master computer:
```
roscore
```
Connect to Zeus and source to the master computer and advertise services:
```
ssh robot@192.168.1.10
. ./setup_wam.sh
roslaunch wam_bringup wam_bringup.launch
```
Connect to Slax, source to the master computer and advertise services:
```
ssh robot@192.168.1.11
source scripts/ips_USER.txt
rosrun wam_node wam_node
```
### Linking Arms
Run from the master computer:
```
rosservice call /zeus/wam/link_arm 192.168.1.11
rosservice call /slax/bhand/link_arm 192.168.1.10
```
### Connecting Arduino Wrist
Connect the wrist to the master computer via USB. <br />
Run from the master computer:
```
rosrun rosserial_python serial_node.py /dev/ttyACM0
rosrun encoder_to_wam_wrist encoder_to_wrist
```