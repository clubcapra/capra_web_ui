import { Ros, Topic } from 'roslib'
// import ROS2D from 'ros2d/build/ros2d.min'

let instance = null

class RosClient {
  ros = null

  constructor() {
    this.ros = new Ros()
  }

  setListeners({ onConnection, onClose }) {
    this.ros.on('connection', onConnection)
    this.ros.on('close', onClose)
    this.ros.on('error', error => {
      if (process.env.NODE_ENV !== 'production')
        console.error('RosError', error)
    })
  }

  setSubscribers({ updateOrientation, updateTemperature }) {
    this.capraSubscribe('imu', 'sensor_msgs/Imu', ({ orientation }) => {
      updateOrientation(orientation)
    })

    this.capraSubscribe(
      'temp',
      'sensor_msgs/Temperature',
      ({ temperature }) => {
        updateTemperature(temperature)
      }
    )
  }

  connect(robotIP = 'localhost') {
    this.ros.close()
    this.ros.connect(`ws://${robotIP}`)
  }

  disconnect() {
    this.ros.close()
  }

  capraSubscribe(name, messageType, handler) {
    let topic = new Topic({
      ros: this.ros,
      name: '/capra/' + name,
      messageType
    })

    topic.subscribe(handler)
  }
}

export default RosClient
