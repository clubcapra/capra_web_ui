import { Ros, Topic, Message } from 'roslib'

type TopicOptions = { name: string; messageType: string }

class RegisteredTopic {
  handlers: Array<Function> = []
  listener: Topic | undefined | null
  options: TopicOptions

  constructor(options: TopicOptions, handler: Function) {
    this.handlers = [handler]
    this.listener = undefined
    this.options = options
  }
}

class RosClient {
  ros: Ros
  registeredTopics: Map<string, RegisteredTopic> = new Map()

  getSignature = (options: TopicOptions) => {
    return `${options.name}/${options.messageType}`
  }

  constructor(robotIP?: string) {
    this.ros = new Ros({ url: undefined })
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
    const message = new Message(payload)
    topic.publish(message)
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
