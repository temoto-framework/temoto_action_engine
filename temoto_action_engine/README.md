# temoto_action_engine
ROS independent core tools of Temoto Action Engine

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
g++ my_app.cpp -o my_app `pkg-config --cflags --libs temoto_action_engine`
```

Set the ```LD_LIBRARY_PATH``` so that your application can find the temoto_action_engine library
```bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:'/usr/local/lib'
```