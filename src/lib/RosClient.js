import { Ros, Topic } from 'roslib'

class RosClient {
  ros = null

  constructor() {
    this.ros = new Ros()
  }

  setListeners(actions) {
    this.ros.on('connection', actions.connect)
    this.ros.on('close', actions.disconnect)
    this.ros.on('error', actions.error)

    let imu = new Topic({
      ros: this.ros,
      name: '/vectornav/IMU',
      messageType: 'sensor_msgs/Imu'
    })

    imu.subscribe(({ orientation }) => {
      actions.updateOrientation(orientation)
    })
  }

  connect(url = 'ws://localhost:9090') {
    this.ros.connect(url)
  }
}

export default RosClient
