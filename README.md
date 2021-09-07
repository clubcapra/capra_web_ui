# capra_web_ui

This is a UI application for manually controlling ROS-based robots in real-time. While it is designed for the use case of club capra, we try to make it as robot agnostic as possible. As long as your robots has the required dependencies it should be almost plug and play.

- [capra_web_ui](#capra_web_ui)
  - [Installation](#installation)
  - [npm setup](#npm-setup)
    - [For Linux](#for-linux)
    - [For Windows](#for-windows)
  - [Building](#building)
  - [ROS dependencies](#ros-dependencies)
    - [Audio](#audio)
  - [Global state - redux and xstate](#global-state---redux-and-xstate)

## Installation

We provide installers for Windows and Ubuntu that can be found in the release page. If a release is marked as pre-release it means the features haven't been tested with an actual robot.

## npm setup

<!-- TODO make a dev setup readme or wiki page -->

While you can install node/npm manually, it's easier to use a version manager. This let's you have multiple node version installed without any issues.

### For Linux

We recommend using <https://github.com/nvm-sh/nvm>

### For Windows

We recommend using <https://github.com/coreybutler/nvm-windows>

## Building

capra_web_ui is made with web technologies this means you need node.js installed on your machine to build it. We assume the latest version of nodejs is installed.

- Download and install all required dependencies to build or develop the app.

```shell
npm i
```

- Bundle the typescript in src/main and src/renderer in `/build` and use those to build the electron app installer. The final executable will be found in `/dist`

```shell
npm run build
```

- Start the application in development mode with hot module reloading. This means you can make changes in your editor and they will be instantly visible in the app.

```shell
npm run start
```

## ROS dependencies

- [rosbrige_suite](http://wiki.ros.org/rosbridge_suite) used to communicrate with robot
- [web_video_server](http://wiki.ros.org/web_video_server) used to stream video
- [tf2_web_republisher](https://wiki.ros.org/tf2_web_republisher) used to stream 3d model
- [capra_audio](https://github.com/clubcapra/capra_audio_common)

You need to have a small web server running in the folder containing the robot_description.

For example, for our markhor robot:

```bash
cd catkin_ws/src/markhor
python3 -m http.server 88 # launch a server with python on port 88
```

Note: the port for the robot_description is currently hardcoded to 88. It will be configurable in the future.

After that you need to `rosrun` a robot_description

```shell
rosrun tf2_web_republisher tf2_web_republisher
roslaunch web_video_server web_video_server.launch
roslaunch rosbridge_server rosbridge_websocket.launch
```

### Audio

Currently, the audio IO is handled by the [capra_audio](https://github.com/clubcapra/capra_audio_common) node. The UI will simply launch the node as a child process. This only works on linux. Windows support should work with wsl but is not currently implemented.

## Global state - redux and xstate

For global state handle we use a mix of redux and xstate. While it's technically possible to use xstate context for the entirety of the global state. It's much easier to store global state in redux and only use xstate when the state is an actual state machine and not just pure data.
