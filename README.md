# myKINOVA
Towards building contact rich manipulation frameworks with the Kinova gen3 ultralightweight robots.

## Pre-Requisites
1. Get Visual Studio 2019 by downloading 'vs_community__98951435.1629706453.exe' present in this repository.
2. Install [CMake](https://github.com/Kitware/CMake/releases/download/v3.29.0-rc1/cmake-3.29.0-rc1-windows-x86_64.msi). Please make sure that you select the option of adding CMake to the system PATH as shown below.

<!--- ![CMake_Windows_install_path](https://github.com/user-attachments/assets/17ca13a4-2346-4334-a130-e01ef98f3c4e)--->

3. Install Boost [using these instructions](https://gist.github.com/zrsmithson/0b72e0cb58d0cb946fc48b5c88511da8). In case you run into an error while installing MinGW-w64, please follow the instructions [here](https://code.visualstudio.com/docs/cpp/config-mingw) under the 'Installing the MinGW-w64 toolchain' section.

Follow the instructions under the MSYS link. Once done, go back to the previous page and continue with steps 4-7.

6. Install [Qt OpenSource](https://master.qt.io/new_archive/qt/5.9/5.9.2/qt-opensource-windows-x86-5.9.2.exe)
7. Install [Robotics Library](https://github.com/roboticslibrary/rl/releases/download/0.7.0/rl-0.7.0-msvc-14.1-x64.msi)

### Build Robotics Library
1. Download the [source code and additional examples](https://www.roboticslibrary.org/download) and extract the archives.
2. In a cmd **opened as administrator**:
```console
cmake -E tar x rl-0.7.0.zip
cmake -E tar x rl-examples-0.7.0.zip
cd rl-0.7.0
mkdir build
cd build
cmake -G "Visual Studio 16 2019" -D CMAKE_INSTALL_PREFIX="C:\Program Files\Robotics Library\0.7.0\MSVC\14.1\x64" ..
cmake --build . --config Release --target INSTALL
```
## Unpack myKINOVA
1. Ensure your firewall is turned OFF (Windows Defender Firewall with Advanced Security).
2. Clone this repository via Visual Studio. Make sure you have [Git](https://git-scm.com/download/win) installed on your system. You can refer to section 2 (ignore 2a) and 3 here to learn more on the installation process.
3. If you run into an error while cloning the repository, open powershell or command prompt --> navigate to the project directory and enter
```console
git restore --source=HEAD :/
```

<!--- ![Git_error](https://github.com/user-attachments/assets/e1387838-c04e-4c3b-abf6-ac0464e3db85)--->

4. Use the CMakeLists to set things up : Project -> Configure kortexApiCppExamples
5. Build -> Build all

fin

# Control strategies for single arm

### Saturation function
<img align="right" src="media/tau_ext_limit.jpg" width="80">
In all the strategies listed below, the external torque used to drive the robot is limited by a saturation function. The purpose is to ensure no high torque is transmitted to the low-level controller of the robot. Note that this may have consequences in your application and will need to be adjusted to allow for high-torque or dynamical control behaviors.

<!-- New line here... -->
---
The UDP need not operate at a high frequency, the low-level torque controller of the robot will still function. These control strategies allow to vary the following variables

| Variable            |    Name |
| -----------         | ------- |
| **$q$**$_{des}$     | Desired joint configuration |
| **$\tau$**$_{cmd}$  | Joint torque command |
| **$\tau$**$_{ext}$  | External torque |

## Mode 0 - Impedance control
<img src="media/mode0_impedance.jpg" height="240">

## Mode 1 - Impedance and torque control
<img src="media/mode1_impedance_and_torque.jpg" height="240">

## Mode 2 - Torque control
<img src="media/mode2_torque.jpg" height="240">

## Mode 3 - Gravity compensation
<img src="media/mode3_gravity.jpg" height="240">

Note: Kinematic calibration is needed to ensure no drift.

## Mode 5 - Impedance, torque and gripper control
<img src="media/mode5_impedance_and_torque_and_gripper.jpg" height="240">
