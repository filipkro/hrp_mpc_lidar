#!/bin/bash
# My first script

echo "Make controller python scripts executable"
chmod +x src/simple_controller/src/controller3.py
catkin_make
rosrun simple_controller controller3.py
echo "End of shell
