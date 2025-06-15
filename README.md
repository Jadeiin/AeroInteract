## Environment preparation:
Follow the install instructions above

### NanoOWL + NanoSAM environment configuration

Install PyTorch, TensorRT, transformers, torch2trt, NanoOWL, NanoSAM
torchvision, onnx, matplotlib (for demo), opencv (invisible requirements)

```
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/root/TensorRT-$VERSION/lib
export PATH=$PATH:/root/TensorRT-$VERSION/bin

# NanoOWL engine (use older onnx opset for older version of pytorch)

python3 -m nanoowl.build_image_encoder_engine \
    /root/autodl-tmp/owl_image_encoder_patch32.engine \
    --onnx_opset=15

# NanoSAM engine

trtexec \
    --onnx=/root/autodl-tmp/mobile_sam_mask_decoder.onnx \
    --saveEngine=/root/autodl-tmp/mobile_sam_mask_decoder.engine \
    --minShapes=point_coords:1x1x2,point_labels:1x1 \
    --optShapes=point_coords:1x1x2,point_labels:1x1 \
    --maxShapes=point_coords:1x10x2,point_labels:1x10

trtexec \
    --onnx=/root/autodl-tmp/resnet18_image_encoder.onnx \
    --saveEngine=/root/autodl-tmp/resnet18_image_encoder.engine \
    --fp16
```

### ROS environment configuration

build the ROS package. Change the corresponding camera topic, engine path, text prompt etc. before using.

```
ln -s AeroInteract/tools/*.sh .
cd catkin_ws && catkin_make && cd ..
# use tmux for better logging
bash rosbag.sh; bash vnc.sh
# or use roslaunch
source devel/setup.bash; roslaunch sam_fp rosbag.launch
# add parameters as needed, like
roslaunch sam_fp rosbag.launch bag_file:=out-15.bag
roslaunch sam_fp rosbag.launch bag_file:=1.bag image_topic:=/oak/rgb/image_raw point_cloud_topic:=/oak/points camera_frame:=oak-d-base-frame rviz_config:='$(find sam_fp)/rviz/oak-d.rviz'

```

## SITL: PX4 + Gazebo + MAVROS

The simulation environment uses PX4 SITL with Gazebo and MAVROS, featuring an Iris quadcopter equipped with a RealSense D435i depth camera.

### Prerequisites

1. Install PX4 firmware and build SITL:
```bash
# Clone PX4 firmware
git clone https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot

# Install dependencies
bash Tools/setup/ubuntu.sh

# Build SITL
make px4_sitl gazebo
```

2. Install ROS dependencies:
```bash
sudo apt install ros-${ROS_DISTRO}-gazebo-ros-pkgs ros-${ROS_DISTRO}-mavros ros-${ROS_DISTRO}-mavros-extras
```

3. Install realsense_ros_gazebo
```bash
git clone https://gitee.com/nie_xun/realsense_ros_gazebo
ln -s realsense_ros_gazebo ~/catkin_ws/src
```

### Running the Simulation

1. Build the ROS workspace:
```bash
cd catkin_ws
catkin_make
# Install iris model with depth camera refer to https://github.com/siyuanwu99/iris_D435i_gazebo/blob/main/install.sh
# And do some modifications: turn on pointCloud in D435i's sdf, edit child_link in iris_D435i's sdf.
```

2. Launch the simulation:
```bash
# Replace /path/to/PX4-Autopilot with your PX4 firmware directory
bash tools/traverse_sitl.sh /path/to/PX4-Autopilot
```

This will launch:
- PX4 SITL simulation with Gazebo
- MAVROS for ROS communication with PX4
- Custom world with doors for navigation
- Iris drone equipped with D435i depth camera
- SAM detection nodes for door recognition
- Autonomous navigation node

### Launch File Configuration

The simulation can be configured through launch file arguments:
- `search_text`: Text prompt for SAM detection (default: "[a door]")
- `vehicle_model`: Vehicle model to use (default: iris_D435i)
- `world`: Custom Gazebo world file (default: door.world)
- `gui`: Enable/disable Gazebo GUI
- `x, y, z`: Initial position
- `R, P, Y`: Initial orientation

Example with custom parameters:
```bash
./tools/traverse_sitl.sh /path/to/PX4-Autopilot "vehicle_model:=iris_D435i gui:=true x:=2 y:=4"
```

## Jetson Platform Deployment

Using Docker as base development environment, install modifed [jetson-containers](https://github.com/Jadeiin/jetson-containers) and prepare basic environment at first.

Then run command below:

```
# base package for development usage, camera package for deployment usage
jetson-containers build sam_ros:base --package-dirs=/path/to/AeroInteract/tools

# You can add following arguments to speed-up build steps if your network situation are bad
# --build-arg http_proxy=http://127.0.0.1:7890,https_proxy=http://127.0.0.1:7890
# And skip all tests
# --skip-tests all
# Or you want test all packages, HTTP_PROXY & HTTPS_PROXY can be set
```

## TODO

- [x] publish door location & interaction point/vector (for nav use): use "/objects_marker"
- [x] add trajectory planning
  - [x] simple straight line
  - [ ] advanced planning (e.g. include door size/shape/orientation + minimal snap + prediction)
- [x] add simulation (e.g. Gazebo)
  - [x] add model (quadrotor & depth camera - iris_depth_camera)
  - [x] add world (doors & walls - )
  - [x] run simulation (adjust door physics & planning logic)
    - [x] TF tree problems: 1. wrong transformation between camera_link and map (fixed: orginal pointcloud position is wrong) 2. Lookup been a little bit fast (0.002~3 ahead, ignored)
    - [x] Door interaction problems: 1. inaccurate dynamics caused by ODE 2. add more options like switch from position mode to traverse mode then fallback (currently takeoff only) 3. implement continuous door traverse for demonstration purpose
    - [x] Integrate with control module
  - [x] add more descriptions & tutorials
- [x] real world test

optional:
- [x] add metrics option
- [x] add code documentation
- [ ] add build pipeline
- [ ] integrate masks_msgs package
- [ ] use cuPCL entirely
- [ ] use nodelet
- [ ] migrate to ROS2
