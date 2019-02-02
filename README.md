# takin-ui

User interface for the Takin project. It can be used for any similar robots with the same set of feature using ROS.

It currently uses electron to run, but it isn't necessary for now. This could be a simple web app. We might move to a pure web app if fast deployments become necessary. We try to not directly use electron features for now. In some limited testing we saw some small improvements to video streams, but after some more testing it doesn't seem to do much. The limited target environment is nice, but not necessary for now.

## Technologies used

- Vue.js
  - vuex
  - vue-router
  - electron-vue
- yarn
- electron
- css grid, sass/scss
- bulma for simple css components and we use vue-bulma-components to integrate with vue
- Eslint
- prettier
- webpack
- electron
- roslibjs
- lodash
- font-awesome
- babel

You do not need to be familiar with everything on this list but you should try using these before considering adding a new dependency. **Vue is the most important dependency here and you should know it to work on this project.**

Note: most web ecmascript project uses npm. While we do not use it directly you should already have it install. For package management we use yarn.

## Build/run Setup

```bash
# if yarn is not installed
npm install -g yarn

# install dependencies
yarn

# serve with hot reload at localhost:9080
yarn run dev

# build electron application for production
yarn run build


# lint all JS/Vue component files in `src/`
yarn run lint

```

## ROS dependencies

```bash
sudo apt install ros-kinetic-rosbridge-suite
sudo apt install ros-kinetic-web-video-server

roslaunch rosbridge_server rosbridge_websocket.launch
rosrun web_video_server web_video_server

roslaunch capra_imu imu.launch
roslaunch capra_camera_3d capra_camera_3d.launch
```

---

This project was generated with [electron-vue](https://github.com/SimulatedGREG/electron-vue)@[8fae476](https://github.com/SimulatedGREG/electron-vue/tree/8fae4763e9d225d3691b627e83b9e09b56f6c935) using [vue-cli](https://github.com/vuejs/vue-cli). Documentation about the original structure can be found [here](https://simulatedgreg.gitbooks.io/electron-vue/content/index.html).
