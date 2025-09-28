# Mechatronics Undergraduate Final Project

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
3. Execute the _'init.bat'_ file (automatically starts the server and HMI)
4. Power on the robot and controller (certify that the emergency stop button is released)

## Working
1. The joint values should be updated to the real current measures
2. Press 'START' button to enable the joint control
<img width="1919" height="1079" alt="image" src="https://github.com/user-attachments/assets/fba98134-332e-4356-9cad-8c2ebb70ff8e" />

## Implemented features
* _'Home'_ tab:
  - 'START'/'HOLD' button
  - 'SPD OVR' (partially implemented)
  - 'REC'/'STOP' button
  - 'GO to HOME' button
  - Joint 'real time' values
  - Cartesian 'real time' values
* _'Modeling'_ tab:
  
  **‼️(THIS TAB MUST BE CAREFULLY USED - CAN DAMAGE THE SYSTEM - CONTROL PARAMETERS ARE DIRECTLY SET)‼️**
  - 'Open Loop' mode
  - 'PID' mode
* _'Motion'_ tab:
  - 'Joint' mode
  - 'Target' mode
* _'Program'_ tab:
  - 'New File' button
  - 'Open File' button
  - 'Save File' button
* _'Graph'_ tab:
  - 'Single DOF' mode
  - 'ALL DOFs' mode
  - 'ALL PWMs' mode
  - 'Cartesian' mode

## Remarks
* Dynamic modeling was not done.
* The controller tunning is not the ideal one and must be improved in the future.
* Non-linear control techniques will be required to handle with non-linearities such as deadbands, slashes and intrinsic manipulator model dynamics.
* Only proportional effect of the PID is being used for now.
* Gravity affected joints may present steady state errors.
* Since the wrist is built by a differential arrangement, it may present oscilations and steady state errors.
* The grip is programmed to follow a specific opening target and not to grab any object automatically.

---
#### **⚠️ _THIS PROJECT IS A WORK IN PROGRESS AND MAY CONTAIN ERRORS AND UNFINISHED RESOURCES_ ⚠️**
---
