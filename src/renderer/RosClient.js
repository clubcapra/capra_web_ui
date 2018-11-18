import { Ros, Topic } from 'roslib'

class RosClient {
  ros = null
  registeredTopics = {}

  getSignature = ({ name, messageType }) => `${name}/${messageType}`

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

  setListener(options) {
    const signature = this.getSignature(options)

    const listener = new Topic({
      ros: this.ros,
      options
    })

    listener.subscribe(message => {
      this.registeredTopics[signature].handlers.forEach(handler =>
        handler(message)
      )
    })
  }

  subscribe(options, handler) {
    const signature = this.getSignature(options)
    if (signature in this.registeredTopics) {
      this.registeredTopics[signature].handlers.push(handler)
    } else {
      this.registeredTopics[signature] = {
        options,
        listener: null,
        handlers: [handler]
      }

      this.setListener(options)
    }
  }
}

export default RosClient
