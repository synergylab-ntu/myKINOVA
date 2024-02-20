# myKINOVA
Towards building contact rich manipulation frameworks with the Kinova gen3 ultralightweight robots

## Pre-Requisites
1. Install [CMake](https://github.com/Kitware/CMake/releases/download/v3.29.0-rc1/cmake-3.29.0-rc1-windows-x86_64.msi)
2. Get [Visual Studio 2019](https://my.visualstudio.com/Downloads?q=visual%20studio%202019&wt.mc_id=o~msft~vscom~older-downloads)
3. Install Boost [using these instructions](https://gist.github.com/zrsmithson/0b72e0cb58d0cb946fc48b5c88511da8)
4. Install [Qt OpenSource](https://master.qt.io/new_archive/qt/5.9/5.9.2/qt-opensource-windows-x86-5.9.2.exe)
5. Install [Robotics Library](https://github.com/roboticslibrary/rl/releases/download/0.7.0/rl-0.7.0-msvc-14.1-x64.msi)

### Build Robotics Library
1. Download the [source code and additional examples](https://www.roboticslibrary.org/download) and extract the archives.
2. In a cmd:
```console
C:\synergylab\luigi>$ cmake -E tar x rl-0.7.0.zip
luigi@synergylab:~$ cmake -E tar x rl-examples-0.7.0.zip
luigi@synergylab:~$ cd rl-0.7.0
luigi@synergylab:~$ mkdir build
luigi@synergylab:~$ mkdir build
```
