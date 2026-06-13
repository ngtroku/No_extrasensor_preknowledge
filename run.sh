#!/bin/bash

python3 spoofing_simulation.py
python3 gen_spoofed_rosbag.py
python3 gen_mitigate_rosbag.py

python3 post_process.py