#!/bin/bash

# Replace 'your_docker_container' with the name of your Docker container
DOCKER_CONTAINER="myagvsuper"

# Attempt to start the Docker container
docker start $DOCKER_CONTAINER 2>/dev/null

# Start a new terminal window and run the Docker container
gnome-terminal -- bash -c "
    # Run the Docker container in interactive mode with a TTY
    docker exec -it $DOCKER_CONTAINER bash -c '
        # Set a trap to run pkill when the terminal is closed
        trap \"pkill roscore\" EXIT;

        # Source the ROS environment and start roscore
        source /opt/ros/noetic/setup.bash;
        roscore;
    '
"

# Keep the terminal open
wait

