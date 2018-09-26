import { Ros, Topic } from 'roslib'
// import ROS2D from 'ros2d/build/ros2d.min'

let instance = null

class RosClient {
  ros = null

  constructor() {
    if (!instance) instance = this

    this.ros = new Ros()

    return instance
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

  initMap2D(divID) {
    console.log('initMap2D: ' + divID)
    //console.log(ROS2D)
    //eslint-disable-next-line
    // let viewer = new ROS2D.Viewer({
    //   divID: divID,
    //   width: 400,
    //   height: 400
    // })
    // console.log(viewer)

    // let gridClient = new ROS2D.OccupancyGridClient({
    //   ros: this.ros,
    //   rootObject: viewer.scene
    // })

    // gridClient.on('change', () => {
    //   viewer.scaleToDimensions(
    //     gridClient.currentGrid.width,
    //     gridClient.currentGrid.height
    //   )
    // })
  }
}

export default RosClient
