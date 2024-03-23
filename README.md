# UDS Kobuki ROS

## Windows

[Install ROS2 on Windows 11 guide](https://ms-iot.github.io/ROSOnWindows/GettingStarted/SetupRos2.html)

Switch on developer mode for use of ROS2 symlink install.

Settings -> System -> For developers -> Developer mode to On

## Setup VSCode

1. File->Preferences->Settings
2. Seatch for "terminal.integrated.profiles.windows" property and click "Edit in settings.json"
3. Add this to the json file under "terminal.integrated.profiles.windows"

    ```
    "terminal.integrated.profiles.windows": {
        "DevCmd": {
            "path": [
                "${env:windir}\\Sysnative\\cmd.exe",
                "${env:windir}\\System32\\cmd.exe"
            ],
            "args": [
                "/d",
                "/k", 
                "C:\\Program Files (x86)\\Microsoft Visual Studio\\2019\\Community\\VC\\Auxiliary\\Build\\vcvarsall.bat",
                "amd64"]
        },
    }
    ```
4. Open the DevCmd terminal in TERMINAL tab on right side with arrow next to plus sign
5. Optional: Make DevCmd terminal default on Windows

    ```
    "terminal.integrated.defaultProfile.windows": "DevCmd"
    ```

## Setup and build package

1. Source ROS2 `c:\opt\ros\foxy\x64\setup.bat`
2. Create ROS2 Workspace `mkdir ros2_ws\src`
3. Clone project to the workspace

    ```
    cd ros2_ws\src
    git clone ...
    ```
4. Build `uds_kobuki_ros` package

    ```
    cd ..
    colcon build --symlink-install --merge-install --packages-select uds_kobuki_ros
    call install/local_setup.bat
    ```
5. Run `uds_kobuki_ros`

    ```
    ros2 run uds_kobuki_ros uds_kobuki_ros
    ```

6. Install `uds_kobuki_ros`

    ```
    colcon build --symlink-install --merge-install --packages-select uds_kobuki_ros --install-base c:\opt\install
    ```