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
      name: '/capra/imu',
      messageType: 'sensor_msgs/Imu'
    })

    imu.subscribe(({ orientation }) => {
      actions.updateOrientation(orientation)
    })

    let temperature = new Topic({
      ros: this.ros,
      name: '/capra/temp',
      messageType: 'sensor_msgs/Temperature'
    })

    temperature.subscribe(({ temperature }) => {
      actions.updateTemperature(temperature)
    })
  }

  connect(robotIP = 'localhost') {
    this.ros.connect(`ws://${robotIP}:9090`)
  }
}

export default RosClient
