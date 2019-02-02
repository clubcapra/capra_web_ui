# takin-ui

User interface for the Takin project. It can be used for any similar robots with the same set of feature using ROS.

It currently uses electron to run, but it isn't necessary for now. This could be a simple web app. We might move to a pure web app if fast deployments become necessary. We try to not directly use electron features for now. In some limited testing we saw some small improvements to video streams, but after some more testing it doesn't seem to do much. The limited target environment is nice, but not necessary for now.

## Technologies used

- Vue.js (currently vue 2 but when vue 3 comes out we might upgrade)
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

You can look in project.json for more specific dependencies.

**Vue is the most important dependency here and you should know it to work on this project.**

You do not need to be familiar with everything on this list but you should try using these before considering adding a new dependency. We try not to add dependency for simple functionalities, try to either write it yourself or use already used dependencies. lodash provides a lot of utility functions not provided by the small javascript standard library.

Note: most javascript projects uses npm. While we do not use it directly you should already have it installed. For package management we use yarn.

## Folder structure

src is essentially the root folder everything else is configuration files that shouldn't be touched as much as possible.

- main: contains code that runs on the node runtime. This is specific to electron. For now we try to not use any features not available on the web.
- renderer: this is essentially the src folder of a normal web app. Everything in here should assume the context of a browser window.
  - assets: contains any artifacts that aren't code. For examples: images or theme files that are loaded but not modified. If you want a custom theme you should either use the style feature of vue or make a new theme folder, since we use bulma wetry not to invent new themes and use what already exists.
  - components: contains all vue components used to build the application. The internal structure isn't defined for now, but we try to sort them by features and not type.
  - store: contains vuex modules and other utility functions related to using these stores
  - App.vue: main entry point of the vue app
  - main.js: main entry point of the entire web app. This should rarely change.
  - router.js: Since our router is really simple we only have a single file. If it becomes more complex we will make a router folder
  - RosClient.js: small utility library that aims to simplify using roslibjs

Vue is a component based framework, this means we try to use composition as much as possible. Since this is a vue app we try to use vue features and the vue way of doing things.

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
