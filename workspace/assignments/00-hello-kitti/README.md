# Hello KITTI
Demo used to test the environment

## Build
Open a terminal in the docker environment
```bash
cd /workspace/assignments/00-hello-kitti
catkin config --install
catkin build
```

## Run
```bash
source install/setup.bash
roslaunch lidar_localization test_frame.launch
```