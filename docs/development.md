# Development

## Windows

[Install ROS2 using guide in README.md](../README.md#install-ros2-foxy)

Switch on developer mode for use of ROS2 symlink install.

Settings -> System -> For developers -> Developer mode to On

### Setup VSCode

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

