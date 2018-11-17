import { Ros, Topic } from 'roslib'
// import ROS2D from 'ros2d/build/ros2d.min'

const instance = null

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

  connect(robotIP = 'localhost:9090') {
    this.ros.close()
    this.ros.connect(`ws://${robotIP}`)
  }

  disconnect() {
    this.ros.close()
  }

  subscribe({ name, messageType }, handler) {
    const topic = new Topic({
      ros: this.ros,
      name,
      messageType
    })

    topic.subscribe(handler)
  }
}

export default RosClient
