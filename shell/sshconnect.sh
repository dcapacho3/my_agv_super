#!/bin/bash

# Set the variables
USERNAME="david"
PASSWORD="ubuntu"
HOST="localhost"
COMMAND="ros2 topic list"  # or any other command you want to run

# Use sshpass to connect to the remote host
sshpass -p "$PASSWORD" ssh -t -o "StrictHostKeyChecking=no" "$USERNAME"@"$HOST" << EOF
  # Use a login shell to ensure .bashrc is sourced
  exec bash --login << INNEREOF
    # Print debug information
    echo "Current shell: \$SHELL"
    echo "Current user: \$(whoami)"
    echo "Home directory: \$HOME"
    echo "Initial PATH: \$PATH"
    
    # Explicitly set ROS_DISTRO
    export ROS_DISTRO=humble
    
    # Source ROS 2 setup files
    echo "Sourcing ROS 2 Humble setup..."
    source /opt/ros/humble/setup.bash
    source /usr/share/gazebo/setup.bash
    source /usr/share/gazebo-11/setup.bash
    source ~/ros-humble-ros1-bridge/install/local_setup.bash
    
    # Set other environment variables
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export TURTLEBOT3_MODEL=waffle
    export ROS_DOMAIN_ID=30
    export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:/home/david/tesis/superdev_ws/src/my_agv_super/models/
    
    # Print updated environment information
    echo "ROS_DISTRO: \$ROS_DISTRO"
    echo "ROS_VERSION: \$ROS_VERSION"
    echo "Updated PATH: \$PATH"
    echo "RMW_IMPLEMENTATION: \$RMW_IMPLEMENTATION"
    echo "TURTLEBOT3_MODEL: \$TURTLEBOT3_MODEL"
    echo "ROS_DOMAIN_ID: \$ROS_DOMAIN_ID"
    echo "GAZEBO_MODEL_PATH: \$GAZEBO_MODEL_PATH"
    
    # Try to find ros2 executable
    echo "Searching for ros2 executable:"
    which ros2 || echo "ros2 not found in PATH"
    
    # Run the initial ROS2 command
    echo "Attempting to run initial command:"
    $COMMAND
    
    # Keep the session open indefinitely
    while true; do
     # echo "SSH session is active. Press Ctrl+C to exit."
    
    
    #  echo "Enter a command to execute (or press Enter to refresh):"
      read -r user_command
      if [ -n "\$user_command" ]; then
        eval "\$user_command"
      else
        #echo "Session still active. Waiting for command..."
      fi
      sleep 1
    done
INNEREOF
EOF
