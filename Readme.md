# Manus SDK Minimal Client (Windows) - UDP Teleop Broadcaster

This project is a customized, minimal C++ client built on top of the Manus SDK v3.1.1. It is designed to run on Windows and continuously stream hand tracking and ergonomics (finger joint) data over a UDP connection. 

This is particularly useful for robotic teleoperation (e.g., streaming MANUS Glove data to a ROS2 master or other robotics middlewares like Ubuntu/Linux).

## Overview

The client connects locally to the **Manus Core** software, registers necessary callbacks for tracker and ergonomics streams, and then processes the incoming data in real-time. 

Specifically, it extracts:
1. **Right Wrist Tracker Data:** 
   - 3D Position (`X, Y, Z`)
   - 3D Rotation (Convert Quaternion to Euler Angles: `Roll, Pitch, Yaw`)
2. **Right Glove Ergonomics Data (15 Joints):**
   - Extracts 4 metrics per finger (MCP Spread, MCP Stretch, PIP Stretch, DIP Stretch).
   - Thumb, Index, Middle, Ring, Pinky.

The extracted data is packed into a custom `HandDataPacket` and broadcasted via UDP at ~50Hz.

## Core Files

* `SDKMinimalClient.hpp` / `SDKMinimalClient.cpp`: Contains the main SDK logic, Core connection, Callback handlers, UDP Socket setup, and the main run-loop.
* `ManusSDK/`: Pre-compiled libraries, DLLs, and header files for Manus SDK.
* `.vcxproj` / `.sln`: Visual Studio 2022 project files for building the client.

## Network Configuration

By default, the UDP packets are sent to:
* **Target IP**: `192.168.0.112` (Modify in `SDKMinimalClient.cpp::Run()`)
* **Target Port**: `12345`
* **Frequency**: ~50Hz (20ms sleep per cycle)

## How to Build & Run

### Prerequisites
* Windows 10/11
* Visual Studio 2022 (with Desktop development with C++)
* Manus Core installed and running in the background with a paired right glove and tracker.

### Building via MSBuild / PowerShell
```powershell
cd SDKMinimalClient_Windows
& "C:\Program Files\Microsoft Visual Studio\2022\Community\MSBuild\Current\Bin\MSBuild.exe" SDKMinimalClient.vcxproj /p:Configuration=Release /p:Platform=x64
```
*(Ensure the MSBuild path matches your Visual Studio installation.)*

### Running
Once built successfully, the executable will be placed in the `Output/x64/Release/` folder.
```powershell
.\Output\x64\Release\SDKMinimalClient_Windows.exe
```

Upon execution, the terminal will dynamically display the parsed Right Wrist positional and rotational data, along with the real-time angles of all 15 finger joints. Press `Spacebar` to exit gracefully.
