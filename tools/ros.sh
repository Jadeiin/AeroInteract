#!/bin/bash

# Define the tmux session name
SESSION="ros"

# Create a new tmux session but do not attach to it
tmux new-session -d -s $SESSION
tmux setenv LD_LIBRARY_PATH $LD_LIBRARY_PATH:/root/TensorRT-10.5.0.18/lib
tmux setenv HF_ENDPOINT https://hf-mirror.com
tmux setenv DISPLAY :1

# Create four tmux windows
tmux new-window -t $SESSION:1
tmux new-window -t $SESSION:2
tmux new-window -t $SESSION:3
tmux new-window -t $SESSION:4

# Execute the commands in each window
tmux send-keys -t $SESSION:0 'roscore' C-m
tmux send-keys -t $SESSION:1 'cd autodl-tmp && sleep 5 && rosbag play -l 2024-09-26-19-19-48.bag' C-m
tmux send-keys -t $SESSION:2 'cd catkin_ws && source devel/setup.bash && rosrun sam_fp samros.py "[a door]"' C-m
tmux send-keys -t $SESSION:3 'cd catkin_ws && source devel/setup.bash && rosrun sam_fp pcd_processing_node' C-m
tmux send-keys -t $SESSION:4 'cd catkin_ws && source devel/setup.bash && rosrun sam_fp wall_detection_node' C-m

# Attach to the tmux session
tmux attach-session -t $SESSION