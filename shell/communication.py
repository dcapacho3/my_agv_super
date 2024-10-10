#!/usr/bin/env python3

import subprocess

def main():
    # Command for dockerstarter.sh in the first tab
    docker_tab = ['--tab', '-e', 'bash -c "python3 dockerstarter.py; exec bash"']
    autolidar_tab = ['--tab', '-e', 'bash -c "sleep 4; python3 lidarstarter.py; exec bash"']
    
    # Command for bridgestarter.sh in the second tab with a 2-second delay
    bridge_tab = ['--tab', '-e', 'bash -c "sleep 2; python3 bridgestarter.py; exec bash"']
    
    # Command for autossh.py in the third tab with a 4-second delay
    autossh_tab = ['--tab', '-e', 'bash -c "sleep 4; python3 autossh.py; exec bash"']
    
    # Combine all commands into a single gnome-terminal command
    command = ['gnome-terminal'] + docker_tab + bridge_tab + autossh_tab + autolidar_tab
    
    # Run the gnome-terminal with all tabs
    subprocess.Popen(command)

if __name__ == "__main__":
    main()

