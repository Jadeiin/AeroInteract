#!/bin/bash

# Define the tmux session name
SESSION="ros"
export DISPLAY=:1

# Create a new tmux session but do not attach to it
tmux new-session -d -s $SESSION

# Split the tmux window into four panes
tmux split-window -h
tmux split-window -v
tmux select-pane -t 0
tmux split-window -v

# Execute the commands in the split windows
tmux send-keys -t $SESSION:0.0 'roscore' C-m
tmux send-keys -t $SESSION:0.1 'cd autodl-tmp && sleep 5 && rosbag play -l 2024-09-26-19-19-48.bag' C-m
tmux send-keys -t $SESSION:0.2 'cd catkin_ws && source devel/setup.bash && rosrun sam_fp samros.py "[a door]"' C-m
tmux send-keys -t $SESSION:0.3 'cd catkin_ws && source devel/setup.bash && rosrun sam_fp pcd_processing_node' C-m

# Attach to the tmux session
tmux attach-session -t $SESSION