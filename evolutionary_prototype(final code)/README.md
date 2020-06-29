# UR10 Motion Planning Project README
This README describes the setup process for the development environment for the project and also describes how to compile the project's source code. The README assumes that you're executing the commands from this file on a fresh Ubuntu 16.04.4 installation in the given order. The following requirements must be met by the hardware running the Ubuntu installation:

## System Requirements
1. At least 10GB of free space
2. At least 8GB of RAM for compiling the project
3. An x86 processor that's supported by Ubuntu 16.04.4
4. A graphics accelerator supported by Ogre 1.11

## Preparations
1. `sudo apt install git`
2. `sudo apt install checkinstall`
3. `sudo apt-get install libboost-all-dev`

## Compile and Install CMake for Visual Studio Code
1. `cd ~`
2. `git clone https://github.com/Kitware/CMake --branch v3.13.4`
3. `cd CMake/`
4. `./bootstrap && make -j 6` # Where you can change the number to tune the number of cores to be used
5. Use `sudo make install` or use `sudo checkinstall` if you want to be able to uninstall it afterwards

## Compile and Install libccd
1. `sudo apt install m4`
2. `cd ~`
3. `git clone https://github.com/danfis/libccd --branch v2.1`
4. `mkdir build && cd build`
5. `sudo cmake -DENABLE_DOUBLE_PRECISION=ON -DBUILD_SHARED_LIBS=ON ..`
6. `sudo make -j 6` # Tweak depending on number of processor cores and hyperthreading
7. Use `sudo make install` or use `sudo checkinstall` if you want to be able to uninstall it afterwards

## Compile and Install Eigen
*NOTE: Eigen is a template library. No make command is necessary*
1. `cd ~`
2. `git clone https://github.com/eigenteam/eigen-git-mirror`
3. `cd eigen-git-mirror`
4. `git checkout 03577119efbf10c59553a92f183324d99abae85c`
5. `mkdir build && cd build`
6. `sudo cmake ..`
7. Use `sudo make install` or use `sudo checkinstall` if you want to be able to uninstall it afterwards

## Compile and Install FCL
1. `cd ~`
2. `git clone https://github.com/flexible-collision-library/fcl`
3. `cd fcl`
4. `git checkout 6d008b63b801a2c85435c4ae959c28d7538ad270`
4. `mkdir build && cd build`
5. `sudo cmake .. -DBUILD_TESTING=OFF`
6. `sudo make -j 6` # Tweak depending on number of processor cores and hyperthreading
7. Use `sudo make install` or use `sudo checkinstall` if you want to be able to uninstall it afterwards

## Compile and Install KDL
1. `cd ~`
2. `git clone https://github.com/orocos/orocos_kinematics_dynamics`
3. `cd orocos_kinematics_dynamics/orocos_kdl`
4. `git checkout c994a6f7d3b6db93470787d820422c7e804fcb41`
5. `mkdir build && cd build`
6. `sudo cmake ..`
7. `sudo make -j 6` # Tweak depending on number of processor cores and hyperthreading
8. Use `sudo make install` or use `sudo checkinstall` if you want to be able to uninstall it afterwards

## Compile and Install OIS (Required by Ogre3D)
1. `cd ~`
2. `git clone https://github.com/wgois/OIS --branch v1.5`
3. `cd OIS`
4. `mkdir build && cd build`
5. `sudo cmake ..`
6. `sudo make -j 6` # Tweak depending on number of processor cores and hyperthreading
7. Use `sudo make install` or use `sudo checkinstall` if you want to be able to uninstall it afterwards

## Compile and Install Ogre3D
1. `cd ~`
2. `git clone https://github.com/OGRECave/ogre --branch v1.11.5`
3. `sudo apt install libgles2-mesa-dev libxt-dev libxaw7-dev nvidia-cg-toolkit libsdl2-dev doxygen`
4. `sudo apt install libfreetype6-dev libfreeimage-dev libzzip-dev libxrandr-dev libxaw7-dev freeglut3-dev libgl1-mesa-dev libglu1-mesa-dev nvidia-cg-toolkit libois-dev libboost-thread-dev doxygen graphviz libcppunit-dev`
5. `cd ogre`
6. `mkdir build && cd build`
7. `sudo cmake ..`
8. `sudo cmake --build . --config release -- -j 6` # Tweak depending on number of processor cores and hyperthreading
9. Use `sudo make install` or use `sudo checkinstall` if you want to be able to uninstall it afterwards

## Compile and Install OgreProcedural
1. `cd ~`
2. `git clone https://github.com/OGRECave/ogre-procedural`
3. `cd ogre-procedural`
4. `git checkout 900320c281e283348608ee973e35ca57fa21733c`
5. `mkdir build && cd build`
6. `sudo cmake .. -DCMAKE_CXX_FLAGS=-std=c++11 -DOGRE_HOME=/usr/local/lib/OGRE`
7. `sudo make -j 6` # Tweak depending on number of processor cores and hyperthreading
8. `sudo make install` # NOTE: it will give an error about FindOIS, but it will still work

## Build and Run throwaway_prototype / evolutionary_prototype
1. Install Visual Studio Code for x32 or x64, depending on the machine's processor, using the .deb installer for Ubuntu found in https://code.visualstudio.com/
2. Run Visual Studio Code, go to the Extensions tab and install the C/C++, CMake, CMake Tools and CMake Tools Helper extensions.
3. Copy the project's source code folder to the home directory of the Ubuntu installation
4. In the project's folder open the file resources.cfg and change `emilxp` to the current user's name. This is so that Ogre3D can find the robot's meshes.
4. Restart Visual Studio Code.
5. Open the project's folder in Visual Studio Code.
6. Visual Studio Code will ask if you want to to configure the project. Click yes. If the popup disappears, click `Ctrl+Shift+P` and search for `CMake: Configure`. Pick GCC 5.4.0 as the compiler.
7. From the bottom bar change the build from Debug to Release.
8. Click `Ctrl+Shift+P`, find the `CMake: Build` command and run it.
9. From a new terminal window navigate to the build directory in the project's folder and run `./cmake_project`.
