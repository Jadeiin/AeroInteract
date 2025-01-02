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
ln -s sam_ros/tools/*.sh .
cd catkin_ws && catkin_make && cd ..
# use tmux for better logging
bash ros.sh; bash vnc.sh
# or use roslaunch
source devel/setup.bash; roslaunch sam_fp rosbag.launch
# add parameters as needed, like
roslaunch sam_fp rosbag.launch bag_file:=out-15.bag
roslaunch sam_fp rosbag.launch bag_file:=1.bag image_topic:=/oak/rgb/image_raw point_cloud_topic:=/oak/points camera_frame:=oak-d-base-frame rviz_config:='$(find sam_fp)/rviz/oak-d.rviz'

```

## Jetson Platform Deployment

Using Docker as base development environment, install modifed [jetson-containers](https://github.com/Jadeiin/jetson-containers) and prepare basic environment at first.

Then run command below:

```
# base package for development usage, camera package for deployment usage
jetson-containers build sam_ros:base --package-dirs=/path/to/sam_ros/tools

# You can add following arguments to speed-up build steps if your network situation are bad
# --build-arg http_proxy=http://127.0.0.1:7890,https_proxy=http://127.0.0.1:7890
# And skip all tests
# --skip-tests all
# Or you want test all packages, HTTP_PROXY & HTTPS_PROXY can be set
```

## TODO

- [x] add metrics option
- [x] add code documentation
- [ ] move masks_msgs package to sam_fp
- [ ] use nodelet
- [ ] migrate to ROS2