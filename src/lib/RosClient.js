import { Ros } from 'roslib'

class RosClient {
  ros = null

  constructor() {
    this.ros = new Ros()
  }

  setListeners(actions) {
    this.ros.on('connection', actions.connect)
    this.ros.on('close', actions.disconnect)
    this.ros.on('error', actions.error)
  }

  connect(url = 'ws://ubuntu:9090') {
    this.ros.connect(url)
  }
}

export default RosClient
