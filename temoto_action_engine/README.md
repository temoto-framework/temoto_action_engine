# temoto_action_engine
ROS independent core tools of Temoto Action Engine

## Dependencies
```bash
sudo apt install libboost-all-dev
sudo apt install libclass-loader-dev
```

## Installation
```bash
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

## Run the demos
There are couple of UMRF graphs in the ```examples/umrf_graphs``` folder which use the ```ta_example_1``` action.
```bash
cd build
./tae_example --actions-path actions --umrf-graph ../examples/umrf_graphs/example_1.umrfg.json
```