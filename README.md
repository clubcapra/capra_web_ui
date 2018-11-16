# takin-ui

> Web UI for ros based robots

## Build Setup

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
