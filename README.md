# Mechatronics Final Project

Development of a microcontroller based controller for a 5 DOF didatic manipulator robot and gripper.

[FINAL PAPER](https://drive.google.com/file/d/1IV7_ScXzZWTYZx_DazngR-hfs36UWTP1/view?usp=drive_link)

## Requirements

* [Microsoft .NET SDK 6.0.428](https://dotnet.microsoft.com/pt-br/download/dotnet/6.0)
* Notebook with WiFi Hotspot
* [Python](https://www.python.org/) (optional - alternative programming)
* [VS Code](https://code.visualstudio.com/) (optional - alternative programming)

## Setup

1. Download the IHM complete folder
2. Enable the WiFi Hotspot and set:
   - SSID: Erick
   - Password: fanplate
3. Execute the 'init.bat' file (automatically starts the server and HMI)
4. Power on the robot and controller (certify that the emergency stop button is released)

## Working

1. The joint values should be updated to the real current measures
2. Press 'START' button to enable the joint controls
<img width="1919" height="1079" alt="image" src="https://github.com/user-attachments/assets/fba98134-332e-4356-9cad-8c2ebb70ff8e" />

## Implemented features

* HOME tab:
  - 'START'/'HOLD' button
  - 'SPD OVR' (partially implemented)
  - 'REC'/'STOP' button
  - 'GO to HOME' button
  - Joint 'real time' values
  - Cartesian 'real time' values
