## Environment preparation:
Follow the install instructions above

### NanoOWL + NanoSAM envirnoment configuration

Install PyTorch, TensorRT, transformers, torch2trt, NanoOWL, NanoSAM
torchvision, matplotlib, onnx, opencv (invisible requirements)

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

### ROS envirnoment configuration

build the ROS package. Change the corresponding camera topic, engine path, text prompt etc. before using.

```
ln -s sam_ros/tools/*.sh .
cd catkin_ws && catkin_make && cd ..
bash ros.sh; bash vnc.sh
```