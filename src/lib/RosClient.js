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

  connect(actions, robotIP = 'localhost') {
    this.ros.connect(`ws://${robotIP}:9090`)

    let getCameraURL = (topic, ip = robotIP) =>
      `http://${ip}:8080/stream?topic=${topic}`

    let camera = {
      front: {
        depth: getCameraURL('/capra/camera_3d/depth/image'),
        thermal: '',
        rgb: getCameraURL('/capra/camera_3d/rgb/image')
      },
      back: {
        depth: '',
        rgb: ''
      }
    }

    actions.updateCamera(camera)
  }
}

export default RosClient
