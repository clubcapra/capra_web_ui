import { TopicOptions, ServiceOptions } from './@types'
import TopicManager from './TopicManager'
import ServiceManager from './ServiceManager'
import ROSLIB from 'roslib'
import { log } from '@/renderer/logger'

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

    const rosInstance = new ROSLIB.Ros({
      transportLibrary: 'websocket',
      // WARN this could potentially cause major issues
      // eslint-disable-next-line @typescript-eslint/ban-ts-comment
      // @ts-ignore
      encoding: 'ascii',
    })
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
      log.info(`RosClient: Connecting to ${url}`)
      this.ros.connect(url)
    } catch (e) {
      if (this.isLogEnabled) {
        log.error(`RosClient: ros failed to connect to ${url} ${e}`)
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
    log.info('RosClient: subscribing to ', options.name)
    this.topicManager.subscribe<{ data: T }>(options, handler)
  }

  subscribeNoData<T>(options: TopicOptions<T>, handler: (message: T) => void) {
    log.info('RosClient: subscribing to ', options.name)
    this.topicManager.subscribe<T>(options, handler)
  }

  unsubscribe(options: TopicOptions) {
    log.info('RosClient: unsubscribing to ', options.name)
    this.topicManager.unsubscribe(options)
  }

  publish<R>(options: TopicOptions, payload: R) {
    if (this.connected) {
      this.topicManager.publish(options, payload)
    }
  }

  callService<P>(options: ServiceOptions, payload?: P): Promise<unknown> {
    if (this.connected) {
      log.info('RosClient: calling service ', options.name)
      return this.serviceManager.callService(options, payload)
    } else {
      log.warn('RosClient: ros is not connected')
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
      log.info('RosClient: ros connected to ', this.robotIP)
      onConnection()
    }
  }

  private onClose(onClose: () => void) {
    return () => {
      this.topicManager.unsubscribeAllTopics()
      this.connected = false
      log.info('RosClient: ros disconnected')
      onClose()
    }
  }

  private onError(onError: (error: unknown) => void): (event: unknown) => void {
    return (error) => {
      this.connected = false
      if (this.isLogEnabled) {
        log.error('RosClient: ros error', error)
      }

      onError(error)

      if (this.options.shouldTryToReconnect) {
        this.connect(this.robotIP, this.port)
      }
    }
  }

  get isLogEnabled() {
    return window.preloadApi.isDev && this.options.enableLogging
  }

  get isConnected() {
    return this.connected
  }
}
