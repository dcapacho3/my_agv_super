#!/bin/bash


# Open a new terminal window and run the scripts in order
gnome-terminal -- bash -c "
    ./dockerstarter.sh;  
    sleep 2
    exec bash
"  &
gnome-terminal -- bash -c "
    ./bridgestarter.sh;  
    sleep 4
    exec bash
"  &

gnome-terminal -- bash -c "
    sleep 4
    python3 autossh.py;  

    exec bash
"  &
wait
