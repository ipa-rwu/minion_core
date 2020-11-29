# minion_core
hardware

```
sudo chmod a+rw /dev/ttyACM0
rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=11520
```

try to pin arduino to ros
```
stty 57600 -F /dev/ttyACM0 raw -echo
```
