# takin-ui

> A web based UI for ros

## Build Setup

```bash
# install dependencies
npm install

# serve with hot reload
npm run serve

# build for production with minification
npm run build
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
