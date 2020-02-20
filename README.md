# capra-web-ui [![Build Status](https://travis-ci.com/clubcapra/capra_web_ui.svg?branch=master)](https://travis-ci.com/clubcapra/capra_web_ui)

Web based user interface for ROS based robots.

## Table of contents

- [Table of contents](#table-of-contents)
- [Technologies used](#technologies-used)
- [Folder structure](#folder-structure)
- [New dev info](#new-dev-info)
- [vscode](#vscode)
  - [Recommended vscode extensions](#recommended-vscode-extensions)
  - [Recommended vscode settings based on the previous extensions](#recommended-vscode-settings-based-on-the-previous-extensions)
- [Project setup](#project-setup)
  - [Available Scripts](#available-scripts)
  - [Learn More](#learn-more)
- [ROS dependencies](#ros-dependencies)

## Technologies used

- Typescript
- Electron
- React
- Redux
  - redux-starter-kit with immer
- styled-components
- css grid
- eslint
- prettier
- webpack
- roslibjs
- lodash

You can look in project.json for more specific dependencies.

## Folder structure

src is essentially the root folder everything else is configuration files that shouldn't be touched unless necessary.

- **src**:
  - **assets**: contains any artifacts that aren't code. For examples: images or theme files that are loaded but not modified.
  - **components**: contains all the components used to build the application. The internal structure isn't defined for now, but we try to sort them by features.
  - **globalStyles**: contains theme definitions and a typed `styled` helper for styled-components
  - **store**: contains redux modules and other utility functions related to using these stores
  - **utils**:
    - **gamepad**: wrapper to simplify using the gamepad api and also does the input handling
    - **math**: math stuff like Vecor2/Vector3
    - **ros**: RosClient is a wrapper for the roslibjs library to simplify usage
    - **hooks**:  contains custom hooks to simplify development
  - **App.tsx**: main entry point of the react app
  - **index.tsx**: main entry point of the entire web app. This should rarely change.

## New dev info

React is a component based framework, this means we try to use composition as much as possible. Since this is a react app we try to use react features and the react way of doing things. We use react hooks and functional components only. Class components should be avoided unless necessary. We try to be functional as much as possible <https://en.wikipedia.org/wiki/Functional_programming>.

If you haven't setup your environment to format your code with eslint on every save, make sure to run `npm run lint` before committing. It will standardize the formatting and line endings for everyone.

## vscode

It is highly recommended to use [Visual Studio Code](https://code.visualstudio.com/).

### Recommended vscode extensions

- Auto Rename Tag
- Bracket Pair Colorizer 2
- Color Highlight
- EditorConfig for VS Code (if you don't use vscode, make sure editorconfig is supported in your editor)
- ESLint
- GitLens
- Prettier - Code formatter
- vscode-styled-components
- (I recommend using any icon pack of you choice because the default one aren't really varied)

### Recommended vscode settings based on the previous extensions

```json
{
  "editor.formatOnSave": true,
  "eslint.alwaysShowStatus": true,
  "eslint.autoFixOnSave": true,
  "eslint.enable": true,
  "eslint.options": {
    "extensions": [".html", ".js", ".jsx", ".ts", ".tsx"]
  },
  "eslint.run": "onType",
  "eslint.validate": [
    "javascript",
    "javascriptreact",
    {
      "autoFix": true,
      "language": "typescript"
    },
    {
      "autoFix": true,
      "language": "typescriptreact"
    },
  ],
  "javascript.format.enable": false,
  "javascript.validate.enable": false,
  "javascript.updateImportsOnFileMove.enabled": "always",
  "typescript.updateImportsOnFileMove.enabled": "always",
  "javascript.preferences.importModuleSpecifier": "non-relative",
  "typescript.preferences.importModuleSpecifier": "non-relative",
}
```

## Project setup

This project was bootstrapped with [Create React App](https://github.com/facebook/create-react-app).

If it's your first time working on a javascript project make sure you have [node/npm](https://nodejs.org/en/) installed. Once it's done you need to run `npm install` or `npm i` to download all the dependencies. You should run this every time `package.json` has changed, it's easier to run it everytime you pull changes from another branch.

### Available Scripts

In the project directory, you can run:

#### `npm start`

Runs the app in the development mode and opens a new electron window.

The page will reload if you make edits.
You will also see any lint errors in the console.

#### `npm test`

Launches the test runner in the interactive watch mode.

#### `npm run build`

Builds the app for production to the `build` folder.
It correctly bundles React in production mode and optimizes the build for the best performance.

The build is minified and the filenames include the hashes.

Also builds an electron app installer for linux and windows in the folder `/dist`.

#### `npm run eject`

**Note: this is a one-way operation. Once you `eject`, you can’t go back!**

If you aren’t satisfied with the build tool and configuration choices, you can `eject` at any time. This command will remove the single build dependency from your project.

Instead, it will copy all the configuration files and the transitive dependencies (Webpack, Babel, ESLint, etc) right into your project so you have full control over them. All of the commands except `eject` will still work, but they will point to the copied scripts so you can tweak them. At this point you’re on your own.

You don’t have to ever use `eject`. The curated feature set is suitable for small and middle deployments, and you shouldn’t feel obligated to use this feature. However we understand that this tool wouldn’t be useful if you couldn’t customize it when you are ready for it.

### Learn More

You can learn more in the [Create React App documentation](https://facebook.github.io/create-react-app/docs/getting-started).

To learn React, check out the [React documentation](https://reactjs.org/).

## ROS dependencies

- [rosbrige_suite](http://wiki.ros.org/rosbridge_suite)
- [web_video_server](http://wiki.ros.org/web_video_server)
- [tf2_web_republisher](https://wiki.ros.org/tf2_web_republisher)

First you need to have a small web server running in the folder containing the robot_description

For example, for our markhor robot:

``` bash
cd catkin_ws/src/markhor

# launch a server with python on port 88
python3 -m http.server 88
```

Note: the port for the robot_description is currently hardcoded to 88. It will be configurable in the future.

After that you need to `rosrun` a robot_description

```bash
rosrun tf2_web_republisher tf2_web_republisher
roslaunch web_video_server web_video_server.launch
roslaunch rosbridge_server rosbridge_websocket.launch
```

<!-- TODO point to launch file -->
