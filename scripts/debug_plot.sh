#!/bin/sh
python transform_monitor.py &
rqt_plot /predict_odom/pose/pose/position:x:y:z &
rqt_plot /extrinsic_lb/pose/position:x:y:z &
rqt_plot /debug/imu_rot/vector/y /debug/laser_rot/vector/y
