# Diabetic Foot Extension

This extension contains modules dedicated to research tasks for diabetic foot monitoring. 

# Prerequisites

* **Operating system**:
    * Ubuntu 18.04.
    * ~~Windows.~~

* **CUDA-Capable GPU**
    * Driver version 418.39 or above.


* **Slicer**
    * It's highly recommend to use the version 4.10.2.

* **Tools & Libraries**:
    * [CMake](http://cmake.org/cmake/resources/software.html)
        * Version 3.5 or above.
    * [Qt](https://www.qt.io/download)
        * Version 5.9.9 or above.
    * [CUDA](https://developer.nvidia.com/cuda-10.1-download-archive-update2)
        * Version 10.1.
    * [Git](http://git-scm.com/downloads)

# Build 

Some modules depend on external libraries or frameworks. For this reason, it is recommended to use the ***SuperBuild* option** that is enabled by default.

## Building on Linux

In Linux is necessary to build PCL library which depends on the following libraries:
* [Eigen](https://eigen.tuxfamily.org/dox/GettingStarted.html)
    * Version 3.3.4 or above.
* [Flann](https://github.com/ahojnnes/flann)
    * Version 1.9.1 or above.
* [Boost](...)
    * Version 1.65 or above.
    * Modules: `Data time`, `Filesystem`, `iostreams`, `MPI`, `Regex` and `Serialization` 

***Note***: In Ubuntu 18.04 you can install dependencies from repository:
``` Bash
$ sudo apt install libeigen3-dev libflann-dev libboost-all-dev
```

...
