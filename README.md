# gazebo_ranger

gazebo_ranger (2023)

![Screenshot from 2024-03-15 17-01-03](https://github.com/HPC-Lab-KOREATECH/gazebo_ranger/assets/157468651/b6549de4-58db-42f6-9e48-996b5e12e011)
![Screenshot from 2024-03-15 17-01-20](https://github.com/HPC-Lab-KOREATECH/gazebo_ranger/assets/157468651/76202934-4f10-474b-aa93-5a92c31663f3)


# Dependencies

```bash
sudo add-apt-repository ppa:borglab/gtsam-release-4.1 && \
sudo apt-get update && \
sudo apt-get upgrade -y && \
sudo apt-get install -y \
    nlohmann-json3-dev \
    libgtsam-dev \
    libgtsam-unstable-dev \
    ros-humble-gazebo-ros-pkgs \
    python3-rosdep2 \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-joint-state-publisher-gui \
    ros-humble-rqt-robot-steering \
    ros-humble-tf2-tools \
    ros-humble-tf-transformations \
    ros-humble-gazebo-* \
    ros-humble-velodyne-gazebo-plugins \
    ros-humble-perception-pcl \
    ros-humble-pcl-msgs \
    ros-humble-vision-opencv \
    ros-humble-xacro \
    ros-humble-tf2-eigen \
    ros-humble-diagnostic-updater \
    ros-humble-geographic-msgs \
    google-mock \
    libboost-all-dev \
    libeigen3-dev \
    libgflags-dev \
    libgoogle-glog-dev \
    liblua5.2-dev \
    libprotobuf-dev \
    libsuitesparse-dev \
    libwebp-dev \
    ninja-build \
    protobuf-compiler \
    python3-pip \
    terminator \
    gedit \
    psmisc \
    cmake \
    libx11-dev \
    xorg-dev \
    libglu1-mesa-dev \
    freeglut3-dev \
    libglew-dev \
    libglfw3-dev

pip install transforms3d utm && \
sudo apt-get autoremove -y && \
sudo apt-get clean && \
sudo rm -rf /var/lib/apt/lists/* && \
pip install -U colcon-common-extensions

cd ~/ros2_ws

rosdep update
rosdep install -i --from-paths src --ignore-src --rosdistro humble -y --skip-keys pcl_1.10 --skip-keys Eigen --skip-keys GTSAM --skip-keys PCL --skip-keys OpenCV
```

# First setup

```bash
cd ~/ros2_ws/src
git clone <package>
cd ~/ros2_ws/src/gazebo_ranger/map/urdf
xacro orchard_geometry.urdf.xacro  > orchard_geometry.urdf
cd ~/ros2_ws && colcon build --symlink-install
```

# With Docker

```bash
cd ~/ros2_ws/src/docker
./run_command.sh

#in docker
cd ~/ros2_ws
mkdir src && cd src
git clone <package>
cd ..

./rosdep_install.sh

colcon build --symlink-install
```

# gazebo simulation 실행

```bash
ros2 launch gazebo_ranger gazebo.launch.py
```
if you launch simulation with rviz
```bash
ros2 launch gazebo_ranger gazebo.launch.py rviz:=true
```

# tf pdf 생성

```bash
ros2 run tf2_tools view_frames
```

# Ranger Control

```bash
# Local path publisher 현재 실행된 global path 중 선택해서 가능 /gym, /straight, /u_turn, /circle
python3 /root/ros2_ws/src/gazebo_ranger/gazebo_ranger/scripts/path_debug/local_path_publisher.py /gym

# 제어 노드 실행 (Local path follower)
ros2 run control car_control
```

# LIO-SAM

```bash
ros2 launch lio_sam run.launch.py
```

# Navigation

This is for navigating in orchard with simulation
You should need previous map_file
```bash
ros2 launch gazebo_ranger slam_nav.launch.py
ros2 launch lio_sam run_loc.launch.py
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True
ros2 run control car_control
```