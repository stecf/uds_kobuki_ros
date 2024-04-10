@echo off

call c:\opt\ros\foxy\x64\setup.bat
call c:\opt\install\setup.bat

ros2 run uds_kobuki_ros uds_kobuki_ros --ros-args -p ip_address:="127.0.0.1"