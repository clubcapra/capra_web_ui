# capra_web_ui

This is a UI application for manually controlling ROS-based robots in real-time. While it is designed for the use case of club capra, we try to make it as robot agnostic as possible. As long as your robots has the required dependencies it should be almost plug and play.

- [capra_web_ui](#capra_web_ui)
  - [Installation](#installation)
  - [Building](#building)
    - [For Linux](#for-linux)
    - [For Windows](#for-windows)
  - [ROS dependencies](#ros-dependencies)

## Installation

We provide installers for Windows and Ubuntu that can be found in the release page. If a release is marked as pre-release it means the features haven't been tested with an actual robot.

## Building

capra_web_ui is made with web technologies this means you need node.js installed on your machine to build it. We assume the latest version of nodejs is installed.

### For Linux

We recommend using <https://github.com/nvm-sh/nvm>

### For Windows

We recommend using <https://github.com/coreybutler/nvm-windows>

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

You need to have a small web server running in the folder containing the robot_description.

For example, for our markhor robot:

```bash
cd catkin_ws/src/markhor

# launch a server with python on port 88
python3 -m http.server 88
```

Note: the port for the robot_description is currently hardcoded to 88. It will be configurable in the future.

After that you need to `rosrun` a robot_description

Run each of these in a separate terminal

```shell
rosrun tf2_web_republisher tf2_web_republisher
roslaunch web_video_server web_video_server.launch
roslaunch rosbridge_server rosbridge_websocket.launch
```

or

```shell
rosrun tf2_web_republisher tf2_web_republisher & roslaunch web_video_server web_video_server.launch & roslaunch rosbridge_server $ rosbridge_websocket.launch
```
