## Environment preparation:
Follow the install instructions of [Segment Anything](https://github.com/facebookresearch/segment-anything) and [CLIP](https://github.com/openai/CLIP)

build the ROS package.

Package based on Tiago robot from Pal. Change the corresponding camera topic etc. before using.

RUN
```
rosrun sam_fp samros.py "beer"
rosrun sam_fp pcd_processing_node
```
where "beer" is the item that you want the CLIP to choose. If no args input, it will segment all the items.
