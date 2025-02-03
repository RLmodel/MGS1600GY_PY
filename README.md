# MGS1600GY_PY
mgs1600gy ROS2 python package


<br/>

## tested environment

- ubuntu 22.04
- ROS2 Humble

<br/>


## clone && build && launch

<br/>

- source ROS2 environment

```bash
source /opt/ros/humble/setup.bash
```

<br/>

- move to {workspace}/{source_folder}

```bash
cd ~/ros2_ws/src
```

<br/>

- clone packages

```bash
git clone https://github.com/RLmodel/MGS1600GY_PY.git
```

<br/>

- move to workspace

```bash
cd ~/ros2_ws
```

<br/>

- colcon build && source install folder

```bash
colcon build --symlink-install --packages-select mgs1600gy_py
source install/setup.bash
```

<br/>

### rules

- move to script folder

```bash
cd ./src/MGS1600GY_PY/mgs1600gy_py/scripts/
```

<br/>

- make rules

```bash
sudo chmod +x create_udev_rules.sh
```

```bash
./create_udev_rules.sh
```

<br/>

### run

- sensor data only

```bash
ros2 run mgs1600gy_py mgs_serial_driver
```

<br/>

- cmd_vel node

```bash
ros2 run mgs1600gy_py mgs_control
```

<br/>

### launch

- sensor data only

```bash
ros2 launch mgs1600gy_py mgs1600gy_driver.launch.py
```

<br/>

- turtlesim test
```bash
ros2 launch mgs1600gy_py turtlesim_test.launch.py
```

<br/>
