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
  - [Code Formatting](#code-formatting)
  - [Release](#release)
  - [Usage Guide](#usage-guide)
    - [Reverse mode](#reverse-mode)
    - [Connecting](#connecting)
    - [Config](#config)
    - [Logging](#logging)

## Installation

We provide installers for Windows and Ubuntu that can be found in the release page. If a release is marked as pre-release it means the features haven't been tested with an actual robot.
<https://github.com/clubcapra/capra_web_ui/releases>

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

Currently, the audio IO is handled by the [capra_audio](https://github.com/clubcapra/capra_audio_common) node. The UI will simply launch the node as a child process. This only works on linux. Windows support should work with wsl but is not currently implemented. For audio to work, you also need to make sure that the capra_audio launch files are sourced in the terminal you are using to launch the UI.

## Global state - redux and xstate

For global state handle we use a mix of redux and xstate. While it's technically possible to use xstate context for the entirety of the global state. It's much easier to store global state in redux and only use xstate when the state is an actual state machine and not just pure data.

## Code Formatting

Our code formatting is handled by Prettier and ESLint. The configuration files for both are located in the root of the project. We recommend using the Prettier and ESLint extensions on VSCode to automatically detect style errors.

To automatically format the code every time you save, add the following to your `settings.json` in the `.vscode` folder:

```json
{
  "editor.codeActionsOnSave": {
    "source.fixAll.eslint": true
  },
  "editor.formatOnSave": true,
  "editor.defaultFormatter": "esbenp.prettier-vscode"
}
```

If you do not have a `settings.json` file or a `.vscode` folder at the root of the project, you can manually create them.

## Release

To create a new release, simply use `npm version [major | minor | patch]`. This will bump the version and create a git tag. You can then push the new commit and github actions will take care of everythin else. You can use `git push --follow-tags` to push the tags to github.

Just make sure to make the github release public once it's done.

## Usage Guide

### Reverse mode

Our markhor robot doesn't have a reverse mode. Instead we simply flip the input sent from the UI to the robot. To activate this revers mode, you can either press the back button of the gamepad or click the button identified either FORWARD or REVERSE to toggle the front direction. When changing the front direction it will also automatically update which camera is focused in the center of the screen. Essentially it will toggle between the camera at the bottom left and the one in the center. It is therefore recommended to configure those cameras with directly front facing and back facing view angles.

### Connecting

To connect to a robot, if you know that the IP is already configured properly, you click on the Disconnected button in the statusbar and it will immediately try to connect. Otherwise, go to the config tab, specify an appropriate IP address and press the connect button.

### Config

All configurations can be found in the config tab. It is separated in multiple sections.

- General: Contains any general purpose configs
- Camera: Configurations related to cameras. In this page you can add as many camera as you have. You can also modify things like the type of camera or the orientation. Every camera added here will be available in the dropdown of each feed.
- Graph: This page is very similar to the camera page, but instead it's to add topics that constantly publish data to be shown in a graph. Like for the cameras, each graph added will be available in any feed.
- Gamepad: Right now this only shows the current mapping of the gamepad. Eventually it will be possible to modify the bindings directly.

### Logging

Since the UI is built on top of electron, there's a sandbox between the web part of the app and the node part of the app. The logs from both parts are configured to log to a file that changes daily.

Logs are written to the following locations:

- Linux: `~/.config/capra_web_ui/logs/{DATE}/{process type}.log`
- Windows: `%USERPROFILE%\AppData\Roaming\capra_web_ui\logs\{DATE}\{process type}.log`

### Testing ROS features locally

To test ROS features without connecting to a robot you simply need to specify `localhost` as the host to connect to while a rosbridge web_socket node is running.

To run the rosbridge server simply open a terminal and run the following command:

`roslaunch rosbridge_server rosbridge_websocket.launch`.

If the package is not installed you can install it:

`sudo apt-get install ros-<rosdistro>-rosbridge-suite`.
