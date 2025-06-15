## Implement NanoOWL and NanoSAM in ROS

##### To launch a world with a table of items and Tiago (for test purpose) (Gazebo)

```
roslaunch sam_fp tiago.launch world_suffix:=tutorial4
roslaunch sam_fp tiago.launch world_suffix:=tutorial4_complex
```

##### To down the Tiago's head

```
rosrun play_motion move_joint head_2_joint -0.7 2.0
```

##### To run sam_fp

```bash
rosrun sam_fp samros.py "[a door]"
rosrun sam_fp pcd_processing_node
```

