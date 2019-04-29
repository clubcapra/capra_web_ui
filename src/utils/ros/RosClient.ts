import { Ros, Topic, Message, Service, ServiceRequest } from 'roslib'
import { TopicOptions } from './types'
import RegisteredTopic from './RegisteredTopic'

class RosClient {
  ros: Ros
  registeredTopics: Map<string, RegisteredTopic> = new Map()

  getSignature = (options: TopicOptions) => {
    return `${options.name}/${options.messageType}`
  }

  constructor(robotIP?: string) {
    this.ros = new Ros({})
    if (robotIP) this.connect(robotIP)
  }

  setListeners(onConnection: Function, onClose: Function, onError: Function) {
    this.ros.on('connection', () => {
      this.reconnectAllDisconnectedHandler()
      onConnection
    })

    this.ros.on('close', () => {
      this.unsubscribeAllTopics()
      onClose()
    })

    this.ros.on('error', error => {
      if (process.env.NODE_ENV !== 'production') {
        console.error('RosError', error)
      }

      // TODO handle unexpected disconnection and reconnect

      onError(error)
    })
  }

  connect(robotIP = 'localhost:9090') {
    this.ros.close()
    this.ros.connect(`ws://${robotIP}`)
  }

  disconnect() {
    this.ros.close()
  }

  listen(options: TopicOptions) {
    const signature = this.getSignature(options)
    const topic = this.registeredTopics.get(signature)

    if (topic) {
      const listener = new Topic({
        ros: this.ros,
        ...options,
      })

      topic.listener = listener

      listener.subscribe(message => {
        topic.handlers.forEach(h => h(message))
      })
    }
  }

  subscribe(options: TopicOptions, handler: Function) {
    const signature = this.getSignature(options)
    const topic = this.registeredTopics.get(signature)

    if (topic) {
      topic.handlers.push(handler)
      return
    }

    this.registeredTopics.set(signature, new RegisteredTopic(options, handler))

    this.listen(options)
  }

  publish(name: string, messageType: string, payload: any) {
    const topic = new Topic({
      ros: this.ros,
      name,
      messageType,
    })

    topic.publish(new Message(payload))
  }

  callService(name: string, serviceType: string, payload: any) {
    const service = new Service({
      ros: this.ros,
      name: name,
      serviceType: serviceType,
    })

    const request = new ServiceRequest(payload)

    return new Promise((resolve, reject) => {
      service.callService(request, resolve, reject)
    })
  }

  unsubscribeAllTopics() {
    this.registeredTopics.forEach(topic => {
      if (topic.listener) {
        topic.listener.unsubscribe()
        topic.listener = null
      }
    })
  }

  reconnectAllDisconnectedHandler() {
    this.registeredTopics.forEach(topic => {
      if (topic.listener === null && topic.handlers.length > 0) {
        this.listen(topic.options)
        topic.listener = null
      }
    })
  }
}

export default RosClient
