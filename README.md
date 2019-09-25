# temoto_action_engine
UMRF format based action execution engine

[![Build Status](https://travis-ci.org/temoto-telerobotics/temoto_action_engine.svg?branch=master)](https://travis-ci.org/temoto-telerobotics/temoto_action_engine)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

For more information TeMoto framework, visit [https://temoto-telerobotics.github.io](https://temoto-telerobotics.github.io).

## Installation
``` bash
git clone --recursive https://github.com/temoto-telerobotics/temoto_action_engine.git
```
or if you want to install Action Engine as a part of the TeMoto framework, then follow [this tutorial](https://temoto-telerobotics.github.io/tutorials/installing_temoto.html).

## Creating TeMoto Actions
Follow [this tutorial](https://temoto-telerobotics.github.io/tutorials/writing_an_action.html) to create TeMoto Actions. NOTE: The tutorial instructs you to create a TeMoto workspace, which simplifies the process of creating actions and launching TeMoto.

## Usage
If you want to use the Action Engine as a node in your ROS project, then use the action_engine_node via
``` bash
rosrun temoto_action_engine action_engine_node <options>
```
where the *options* are
* **-h** [ --help ] - Show help message
* **--mw** arg - Required. Main wake word.
* **--w** arg - Optional. Additional wake words. Indicates to which wake words the action engine will respond to.
* **--a** arg - Optional. Path to action packages path file.
* **--sa** arg - Optional. Path to a single action.
* **--d** arg - Optional. Path to default UMRF that will be executed when the action engine starts up.

The action_engine_node by default subscribes to "*/umrf_graph_topic*" (see the [msg](https://github.com/temoto-telerobotics/temoto_action_engine/blob/master/msg/UmrfJsonGraph.msg) definition) where it's expecting UMRF JSON strings.

If you want to use the Action Engine without other subsystems of the TeMoto Framework, then for now contact the maintainer of this github project.

