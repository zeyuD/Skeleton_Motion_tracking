# Handy Tools for Existing Hand Tracking Interfaces

## About

This tool aims to bridge the gap between the lack of comprehensive ready-to-use examples and existing hand tracking interfaces like:
- [Google MediaPipe Hand](https://developers.google.com/mediapipe/solutions/vision/hand_landmarker)
- [Meta Quest Hand Tracking](https://developer.oculus.com/documentation/unity/unity-handtracking/)
- [Ultraleap Leap Motion Controller](https://developer.leapmotion.com/get-started/)

## Current Organized Progress
- Leap Motion C++ API wrappers
- Calculating and processing position, rotation, and grab
- Socket to send to other platforms
- Save on local machine

## Arranged To Do List
- Limit tracking hand (e.g., only track right hand)
- Display real-time Leap Motion status like FPS

## Implemented but Not Organized and Added
- Hand and body skeleton tracking with Kinect 1&2 (C)
- Hand tracking integration with MediaPipe (Python)
- Hand tracking integration with Meta Quest 2&3 (C#)

## To Be Implemented
- Multi-device calibration

## Tips for WSL (Windows Subsystem for Linux)

### Attaching Leap Motion Controller via USB through WSL

To connect your Leap Motion Controller to WSL, follow these steps:

```bash
# Install usbipd on Windows (run in PowerShell as Administrator)
winget install --interactive --exact dorssel.usbipd-win

# List available USB devices on Windows (PowerShell as Administrator)
usbipd list

# Find the USB device ID
lsusb

# The Leap Motion will appear as "Ultraleap Tracking Camera" or with ID 2936 

# Attach the Leap Motion to WSL (PowerShell as Administrator)
# Replace BUSID with the Bus ID number from the usbipd list output
usbipd bind --busid BUSID
usbipd attach --busid BUSID --wsl

# Verify the device is connected in WSL
lsusb

# Start the Leap Motion service
sudo leapd
```



### Attaching Leap Motion Controller via USB through WSL

```bash
# Location of sample files on Linux system
sudo cp LeapMotion/samples/save_data.c /usr/share/doc/ultraleap-hand-tracking-service/samples/save_data.c
```

Note: The hand tracking UI is unstable in WSL, it is recommended to use command only.