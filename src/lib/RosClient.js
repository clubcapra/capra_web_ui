import { Ros, Topic } from 'roslib'

class RosClient {
  ros = null

  constructor() {
    this.ros = new Ros()
  }

  setListeners({ onConnection, onClose }) {
    this.ros.on('connection', onConnection)
    this.ros.on('close', onClose)
    this.ros.on('error', error => {
      console.error('RosError', error)
    })
  }

  setSubscribers({ updateOrientation, updateTemperature }) {
    let capraSubscribe = (name, messageType, handler) => {
      let topic = new Topic({
        ros: this.ros,
        name: '/capra/' + name,
        messageType
      })

      topic.subscribe(handler)
    }

    capraSubscribe('imu', 'sensor_msgs/Imu', ({ orientation }) => {
      updateOrientation(orientation)
    })

    capraSubscribe('temp', 'sensor_msgs/Temperature', ({ temperature }) => {
      updateTemperature(temperature)
    })
  }

  connect(robotIP = 'localhost') {
    this.ros.connect(`ws://${robotIP}:9090`)
  }

  disconnect() {
    this.ros.close()
  }
}

export default RosClient
