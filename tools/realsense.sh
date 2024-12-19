#!/bin/bash

# Define the tmux session name
SESSION="ros"

# Create a new tmux session but do not attach to it
tmux new-session -d -s $SESSION
tmux setenv HF_ENDPOINT https://hf-mirror.com
#tmux setenv LD_LIBRARY_PATH $LD_LIBRARY_PATH:/root/TensorRT-10.5.0.18/lib

# Create tmux windows
tmux new-window -t $SESSION:1
tmux new-window -t $SESSION:2
tmux new-window -t $SESSION:3

# Execute the commands in each window
tmux send-keys -t $SESSION:0 'roslaunch realsense2_camera rs_rgbd.launch' C-m
tmux send-keys -t $SESSION:1 'cd /workspace/catkin_ws && source devel/setup.bash && rosrun sam_fp samros.py "[a door]"' C-m
tmux send-keys -t $SESSION:2 'cd /workspace/catkin_ws && source devel/setup.bash && rosrun sam_fp pcd_processing_node' C-m
tmux send-keys -t $SESSION:3 'cd /workspace/catkin_ws && source devel/setup.bash && rosrun sam_fp wall_detection_node' C-m

# Attach to the tmux session
tmux attach-session -t $SESSION 