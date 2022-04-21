**TeMoto Action Engine** is a task management system that executes tasks based on UMRF descriptions. Tasks can be started, stopped and modified during runtime.

This repository contains ROS independent *core C++ implementation* of the TeMoto Action Engine. ROS wrappers with respective build instructions and examples available:
* [ROS1](https://github.com/temoto-framework/temoto_action_engine_ros1)
* [ROS2](https://github.com/temoto-framework/temoto_action_engine_ros2)

**More information about**:
* [UMRF](https://doi.org/10.1145/3522580) (paper published in [THRI](https://dl.acm.org/journal/thri))
* [TeMoto Framework](https://github.com/temoto-framework/temoto)

[![Build Status](https://github.com/temoto-framework/temoto_action_engine/actions/workflows/cmake.yml/badge.svg)](https://github.com/temoto-framework/temoto_action_engine/actions/workflows/cmake.yml)
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
There are couple of UMRF graphs in the `examples/umrf_graphs` folder which utilize the `ta_example_1` action. This simple action is designed to increment a number (numerical value + unit), thus it accepts and outputs a parameter named `distance`. In `example_1.umrfg.json`, multiple instances of `ta_example_1` form a loop, and therefore you can see an increasing number being printed out in the terminal:
```bash
cd build
./tae_example --actions-path actions --umrf-graph ../examples/umrf_graphs/example_1.umrfg.json

...

[executeTemotoAction of TaExample1_1] got: 1.000000 meters
[executeTemotoAction of TaExample1_2] got: 2.000000 meters
[executeTemotoAction of TaExample1_1] got: 3.000000 meters
[executeTemotoAction of TaExample1_2] got: 4.000000 meters
[executeTemotoAction of TaExample1_1] got: 5.000000 meters

...
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