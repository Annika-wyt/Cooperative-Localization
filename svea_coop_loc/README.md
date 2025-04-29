## On follower / svea3

```
roslaunch svea_thesis pixy_camera.launch agent_name:=agent7
roslaunch svea_thesis pixy_camera.launch agent_name:=agent8
```

```
roslaunch svea_thesis riccati_localization_occlusion.launch agent_name:=agent7 k:=80 q:=25 x:=0.0 y:=0.4 qw:=0.5 qz:=-1 svea_name:=svea1 lli_port:=/dev/ttyACM1
```

```
roslaunch svea_thesis riccati_localization_occlusion.launch agent_name:=agent8 k:=80 q:=25 x:=-0.4 y:=-0.5 qw:=0.5 qz:=-1 svea_name:=svea3 lli_port:=/dev/ttyACM1
```

```
roslaunch svea_thesis riccati_localization_occlusion.launch agent_name:=agent7 k:=80 q:=25 x:=-0.4 y:=0.4 qw:=0.5 qz:=-1 qx:=0.3 qy:=0.3 svea_name:=svea1 lli_port:=/dev/ttyACM1
```
```
roslaunch svea_thesis riccati_localization_occlusion.launch agent_name:=agent8 k:=80 q:=25 x:=-0.5 y:=0.5 qw:=0.5 qx:=0.3 qy:=0.3 qz:=-1 svea_name:=svea3 lli_port:=/dev/ttyACM0
```
agent number ideally >= 8 
## On leader / svea1

```
roslaunch svea_thesis riccati_localization_occlusion_leader.launch svea_name:=svea1 agent_name:=agent5
```

agent number is linked to the color 
svea id is linked to the svea you are using

### Setup

```
cd src
```

```
git clone --branch multiagent --single-branch https://github.com/Annika-wyt/kth_thesis.git
```

```
git clone https://github.com/KTH-SML/motion_capture_system.git
```

```
git clone https://github.com/aljanabim/nats-ros-connector.git
```

```
cd /src/pixy_camera/include
```

```
git clone https://github.com/charmedlabs/pixy2.git
```

```
cd pixy2/scripts && ./build_libpixyusb2.sh
```

### Run nats locally 

```
docker run --rm -p 4222:4222 nats
```


SetPosition Service:

{
"pose_x" : 10, 
"pose_y" : -20, 
"pose_z" : -5
}

