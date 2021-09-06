import { TopicOptions, ServiceOptions } from './@types'
import TopicManager from './TopicManager'
import ServiceManager from './ServiceManager'
import ROSLIB from 'roslib'

interface Listeners {
  onConnection: () => void
  onClose: () => void
  onError: (error: unknown) => void
}

interface RosClientOptions {
  shouldTryToReconnect?: boolean
  enableLogging?: boolean
  enableSsl?: boolean
}

const defaultOptions: RosClientOptions = {
  shouldTryToReconnect: false,
  enableLogging: false, // TODO add a UI toggle
  enableSsl: false,
}

export default class RosClient {
  ros: ROSLIB.Ros
  private connected = false
  private topicManager: TopicManager
  private serviceManager: ServiceManager
  private robotIP = 'localhost'
  private port = '9090'
  private options: RosClientOptions
  private listeners?: Listeners

  constructor(
    robotIP: string,
    port: string,
    options: RosClientOptions = defaultOptions
  ) {
    this.robotIP = robotIP
    this.port = port
    this.options = options

    const rosInstance = new ROSLIB.Ros({})
    this.ros = rosInstance
    this.topicManager = new TopicManager(rosInstance, this)
    this.serviceManager = new ServiceManager(rosInstance, this)
  }

  setOptions(options: RosClientOptions) {
    this.options = options
  }

  connect(robotIP = this.robotIP, port = this.port) {
    this.robotIP = robotIP
    this.port = port

    const protocol = this.options.enableSsl ? 'wss' : 'ws'
    const url = `${protocol}://${robotIP}:${port}`

    try {
      this.ros.connect(url)
    } catch (e) {
      if (this.isLogEnabled) {
        console.error(`RosClient: ros failed to connect to ${url}`, e)
      }

      if (this.listeners) {
        this.listeners.onError(`RosClient: ros failed to connect to ${url}`)
      }
    }
  }

  disconnect() {
    this.ros.close()
  }

  subscribe<T>(
    options: TopicOptions<T>,
    handler: (message: { data: T }) => void
  ) {
    this.topicManager.subscribe<T>(options, handler)
  }

  unsubscribe(options: TopicOptions) {
    this.topicManager.unsubscribe(options)
  }

  publish<R>(options: TopicOptions, payload: R) {
    if (this.connected) {
      this.topicManager.publish(options, payload)
    } else {
      console.warn('ROS: not connected')
    }
  }

  callService<P>(options: ServiceOptions, payload?: P): Promise<unknown> {
    if (this.connected) {
      return this.serviceManager.callService(options, payload)
    } else {
      console.warn('ROS: not connected')
      return Promise.resolve()
    }
  }

  setListeners({ onConnection, onClose, onError }: Listeners) {
    this.listeners = {
      onConnection,
      onClose,
      onError,
    }

    this.ros.on('connection', this.onConnection(onConnection))
    this.ros.on('close', this.onClose(onClose))
    this.ros.on('error', this.onError(onError))
  }

  private onConnection(onConnection: () => void) {
    return () => {
      this.topicManager.reconnectAllDisconnectedHandler()
      this.connected = true
      onConnection()
    }
  }

  private onClose(onClose: () => void) {
    return () => {
      this.topicManager.unsubscribeAllTopics()
      this.connected = false
      onClose()
    }
  }

  private onError(onError: (error: unknown) => void): (event: unknown) => void {
    return (error) => {
      this.connected = false
      if (this.isLogEnabled) {
        console.error('RosError', error)
      }

      onError(error)

      if (this.options.shouldTryToReconnect) {
        this.connect(this.robotIP, this.port)
      }
    }
  }

  get isLogEnabled() {
    return process.env.NODE_ENV !== 'production' && this.options.enableLogging
  }
}
