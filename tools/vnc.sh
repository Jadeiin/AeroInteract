#!/bin/bash

# Define the tmux session name
SESSION="vnc"

# Create a new tmux session but do not attach to it
tmux new-session -d -s $SESSION
tmux setenv DISPLAY :1

# Split the tmux window into four panes
tmux split-window -h
tmux split-window -v
tmux select-pane -t 0
tmux split-window -v

# Execute the commands in the split windows
tmux send-keys -t $SESSION:0.0 'Xvfb "$DISPLAY" -screen 0 1600x900x24' C-m
sleep 1
tmux send-keys -t $SESSION:0.1 'fluxbox' C-m
tmux send-keys -t $SESSION:0.2 'x11vnc -display "$DISPLAY" -bg -nopw -listen localhost -xkb' C-m
tmux send-keys -t $SESSION:0.3 'rviz' C-m

# Attach to the tmux session
tmux attach-session -t $SESSION