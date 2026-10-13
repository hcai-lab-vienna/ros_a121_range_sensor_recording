#!/bin/bash
. .venv/bin/activate
./record.py --output-file "radaraw_$(date +%Y_%m_%d-%H_%M_%S).h5"
ros2 bag record \
    /camera/camera/color/image_raw \
    /camera/camera/depth \
    /collision_detected \
    /imu \
    /range \
    /raw_bumper_data \
    /rslidar_points
