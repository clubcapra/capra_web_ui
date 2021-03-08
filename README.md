# capra_web_ui

This is a UI application for manually controlling ROS robots in realtime. While it is designed for the use case of club capra, we try to make it as robot agnostic as possible. As long as your robots as the required dependencies it should be almost plug and play.

- [Installation](#installation)
- [Building](#building)
- [ROS dependencies](#ros-dependencies)

## Installation

We provide provide installer for Windows and Ubuntu that can be found in the release page. If a release is marked as pre-release it means the features haven't been tested with an actual robot.

## Building

capra_web_ui is made with web technologies this means you need node.js installed on your machine to build it. Because of a dependency that breaks with npm we are forced to use yarn.

```shell
yarn
```

This will download and install all required dependencies to build or develop the app.

```shell
yarn build
```

This will bundle the typescript in src/main and src/renderer in `/build` and use those to build the electron app installer. The final executable will be found in `/dist`

```shell
yarn start
```

This will start the application in development mode with hot module reloading. This means you can make changes in your editor and they will be instantly visible in the app.

## ROS dependencies

- [rosbrige_suite](http://wiki.ros.org/rosbridge_suite)
- [web_video_server](http://wiki.ros.org/web_video_server)
- [tf2_web_republisher](https://wiki.ros.org/tf2_web_republisher)

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

`rosrun tf2_web_republisher tf2_web_republisher`

`roslaunch web_video_server web_video_server.launch`

`roslaunch rosbridge_server rosbridge_websocket.launch`

or

```shell
rosrun tf2_web_republisher tf2_web_republisher & roslaunch web_video_server web_video_server.launch & roslaunch rosbridge_server $ rosbridge_websocket.launch
```
