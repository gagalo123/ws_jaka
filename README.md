# ws_jaka

# Readme

## Docker

```bash
docker build -t jaka:ubuntu22.04 .
# in the container
find /opt/ros/humble/ -type f -name "launches.py"
mv /opt/ros/humble/lib/python3.10/site-packages/moveit_configs_utils/launches.py /opt/ros/humble/lib/python3.10/site-packages/moveit_configs_utils/launches.py.bak
cp ./src/jaka_ros2/launches.py /opt/ros/humble/lib/python3.10/site-packages/moveit_configs_utils/launches.py
ros2 launch jaka_<robot_model>_moveit_config demo.launch.py use_sim:=true
```

# container run
```bash
xhost +local:docker


docker run -it --rm --gpus all --network host --ipc host -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw -v /dev/dri:/dev/dri --name jaka jaka:ubuntu22.04
docker run -dit --rm --gpus all --network host --ipc host -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw -v /dev/dri:/dev/dri --name jaka jaka:ubuntu22.04
docker run -dit  \
  --gpus all \
  --network host \
  --ipc host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v /dev/dri:/dev/dri \
  -v /home/mofang:/home/mofang \
  --name jaka \
  jaka:ubuntu22.04

```
## jaka
```bash
find /opt/ros/humble/ -type f -name "launches.py"
# mv /opt/ros/humble/lib/python3.10/site-packages/moveit_configs_utils/launches.py /opt/ros/humble/lib/python3.10/site-packages/moveit_configs_utils/launches.py.bak
cp ./src/jaka_ros2/launches.py /opt/ros/humble/lib/python3.10/site-packages/moveit_configs_utils/launches.py
```



### set the user
```bash
getent group docker
sudo usermod -aG docker $USER
newgrp docker
```
