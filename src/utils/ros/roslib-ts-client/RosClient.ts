import { Ros } from 'roslib'
import { TopicOptions, ServiceOptions } from './@types'
import TopicManager from './TopicManager'
import ServiceManager from './ServiceManager'

interface Listeners {
  onConnection: () => void
  onClose: () => void
  onError: (error: unknown) => void
}

interface RosClientOptions {
  shouldTryToReconnect: boolean
  enableLogging: boolean
  enableSsl: boolean
}

const defaultOptions: RosClientOptions = {
  shouldTryToReconnect: false,
  enableLogging: false,
  enableSsl: false,
}

export default class RosClient {
  ros: Ros
  private topicManager: TopicManager
  private serviceManager: ServiceManager
  private robotIP?: string
  private port?: string
  private connected = false
  private options: RosClientOptions
  private listeners?: Listeners

  constructor(
    robotIP = 'localhost',
    port = '9090',
    options: RosClientOptions = defaultOptions
  ) {
    const rosInstance = new Ros({})
    this.ros = rosInstance
    this.topicManager = new TopicManager(rosInstance)
    this.serviceManager = new ServiceManager(rosInstance)

    this.robotIP = robotIP
    this.port = port
    this.options = options
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

  subscribe(options: TopicOptions, handler: Function) {
    this.topicManager.subscribe(options, handler)
  }

  unsubscribe(options: TopicOptions) {
    this.topicManager.unsubscribe(options)
  }

  publish(options: TopicOptions, payload: unknown) {
    this.topicManager.publish(options, payload)
  }

  callService(options: ServiceOptions, payload?: unknown) {
    return this.serviceManager.callService(options, payload)
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

  private onConnection(onConnection: Function) {
    return () => {
      this.topicManager.reconnectAllDisconnectedHandler()
      this.connected = true
      onConnection()
    }
  }

  private onClose(onClose: Function) {
    return () => {
      this.topicManager.unsubscribeAllTopics()
      this.connected = false
      onClose()
    }
  }

  private onError(onError: (error: unknown) => void): (event: unknown) => void {
    return error => {
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

  private get isLogEnabled() {
    return process.env.NODE_ENV !== 'production' && this.options.enableLogging
  }
}
