**TeMoto Action Engine** is a task management system that executes tasks based on UMRF descriptions. Tasks can be started, stopped and modified during runtime.

This repository contains ROS independent core C++ implementation of the Temoto Action Engine. [ROS1](https://github.com/temoto-framework/temoto_action_engine_ros1) or [ROS2](https://github.com/temoto-framework/temoto_action_engine_ros2) wrappers are available.

**For more information about**:
* **UMRF** - (or referring this work) please have a look at our paper: *"[Unified Meaning Representation Format (UMRF) - A Task Description and Execution Formalism for HRI](https://doi.org/10.1145/3522580)"*
* **TeMoto Framework** - [here](https://github.com/temoto-framework/temoto).

[![Build Status](https://travis-ci.org/temoto-telerobotics/temoto_action_engine.svg?branch=master)](https://travis-ci.org/temoto-telerobotics/temoto_action_engine)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

## Install Instructions

```bash
# Get the dependencies
sudo apt install libboost-all-dev
sudo apt install libclass-loader-dev

# Download this repo
git clone --recursive https://github.com/temoto-framework/temoto_action_engine

# Build it
cd temoto_action_engine
mkdir build && cd build
cmake ..
make
sudo make install
```

## Run the demos
There are couple of UMRF graphs in the ```examples/umrf_graphs``` folder which use the ```ta_example_1``` action.
```bash
cd build
./tae_example --actions-path actions --umrf-graph ../examples/umrf_graphs/example_1.umrfg.json
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