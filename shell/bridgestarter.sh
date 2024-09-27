#!/bin/bash

# Run the ros1_bridge dynamic_bridge command
gnome-terminal -- bash -c "
    # Run the command
    ros2 run ros1_bridge dynamic_bridge;

    # Keep the terminal open after the command exits
    exec bash
"
wait
