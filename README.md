**TeMoto Action Engine** is a task management system that executes tasks based on UMRF descriptions. Tasks can be started, stopped and modified during runtime.

This repository contains ROS independent *core C++ implementation* of the TeMoto Action Engine. ROS wrappers with respective build instructions and examples available:
* [ROS1](https://github.com/temoto-framework/temoto_action_engine_ros1)
* [ROS2](https://github.com/temoto-framework/temoto_action_engine_ros2)

**More information about**:
* [UMRF](https://doi.org/10.1145/3522580) (paper published in [THRI](https://dl.acm.org/journal/thri))
* [TeMoto Framework](https://doi.org/10.1109/ACCESS.2022.3173647) (paper published in [IEEE Access](https://ieeeaccess.ieee.org/))

[![Build Status](https://github.com/temoto-framework/temoto_action_engine/actions/workflows/build.yml/badge.svg)](https://github.com/temoto-framework/temoto_action_engine/actions/workflows/build.yml)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

## Install Instructions

```bash
# Get the dependencies
sudo apt install libboost-all-dev nlohmann-json3-dev

# Download this repo
git clone --recursive https://github.com/temoto-framework/temoto_action_engine

# Build it
cd temoto_action_engine
mkdir build && cd build
cmake ..
make
sudo make install
```

## g++ build example
Let's say you have your application, which uses the temoto_action_engine, in ```my_app.cpp```
```cmake
g++ my_app.cpp -o my_app -ltemoto_action_engine
```

Set the ```LD_LIBRARY_PATH``` so that your application can find the temoto_action_engine library
```bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib
```
