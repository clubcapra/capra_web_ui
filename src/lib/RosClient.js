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

    let odom = new Topic({
      ros: this.ros,
      name: '/odom',
      messageType: 'nav_msgs/Odometry'
    })

    odom.subscribe(message => {
      let pose = message.pose.pose
      actions.updatePose(pose)

      let twist = message.twist.twist
      actions.updateTwist(twist)
      // odom.unsubscribe()
    })
  }

  connect(url = 'ws://ubuntu:9090') {
    this.ros.connect(url)
  }
}

export default RosClient
