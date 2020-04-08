# Diabetic Foot Extension

This extension contains modules dedicated to research tasks for diabetic foot monitoring. 

# Prerequisites

* **Operating system**:
    * Ubuntu 18.04.
        * Working and tested using **GCC 7.4**
    * Windows.
        * Working and tested using **MSVC 14.16 (2017 version)**

* **Slicer**
    * It's highly recommend to use the version 4.11 (Nightly version)

* **Tools & Libraries**:
    * [CMake](http://cmake.org/cmake/resources/software.html)
        * Version 3.5 or above.
    * [Boost](https://www.boost.org/)
        * Version 1.65 to 1.69. 
        * Modules: `Data time`, `Filesystem`, `iostreams`, `MPI`, `Regex` and `Serialization` 
    * [Qt](https://www.qt.io/download)
        * Version 5.9.9 or above.
    * [CUDA](https://developer.nvidia.com/cuda-10.1-download-archive-update2)
        * Version 10.1.
    * [Git](http://git-scm.com/downloads)

# Recommendation
* **CUDA-Capable GPU**
    * Driver version 418.39 or above.
    * CUDA Toolkit 10.1
    * cuDNN 7.5

# Build 

Some modules depend on external libraries or frameworks. For this reason, it is recommended to use the ***SuperBuild* option** that is enabled by default.

## Building on Linux

In Linux is necessary to build PCL library which depends on the following libraries:
* [Eigen](https://eigen.tuxfamily.org/dox/GettingStarted.html)
    * Version 3.3.4 or above.
* [Flann](https://github.com/ahojnnes/flann)
    * Version 1.9.1 or above.

***Note***: In Ubuntu 18.04 you can install dependencies from repository:
``` Bash
$ sudo apt install libeigen3-dev libflann-dev libboost-all-dev
```
...

## Building on Windows

1. Download the [Boost Binary](https://sourceforge.net/projects/boost/files/boost-binaries/) corresponding to your MSVC Version and install it
    * It's **highly recommended** to use **MSVC 14.16** (2017 version)
    * Following the Slicer building instructions, probably you have been using the **MSVC 14.0** (2015 version).
1. Set the *``BOOST_ROOT``* global environment variable **or add the entry in CMake** as in the following line:
    ```
        -DBOOST_ROOT:PATH=[Boost Installation Dir]
    ```
1. Add the ``Slicer_DIR`` entry in CMake. 
    * This entry will be used in order to indicate where is Slicer builded. Remember to select the ``Slicer-build`` subdirectory.
    ```
        -DSlicer_DIR:PATH=...
    ```
1. Configure and generate the project.
    * As we said before, we recommend using the **MSVC 14.16**, i.e. the ``v141 toolset``.

1. Click on ``Open Project`` on CMake GUI and build the project.
