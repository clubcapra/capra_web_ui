import { Ros } from 'roslib'
import { TopicOptions, ServiceOptions } from './types'
import TopicManager from './TopicManager'
import ServiceManager from './ServiceManager'

class RosClient {
  private ros: Ros = new Ros({})
  private topicManager: TopicManager = new TopicManager(this.ros)
  private serviceManager: ServiceManager = new ServiceManager(this.ros)
  private robotIP?: string
  private shouldTryToReconnect: boolean

  constructor(robotIP?: string, shouldTryToReconnect: boolean = false) {
    this.shouldTryToReconnect = shouldTryToReconnect

    if (robotIP) {
      this.connect(robotIP)
    }
  }

  connect(robotIP = 'localhost:9090') {
    this.robotIP = robotIP
    this.ros.close()
    this.ros.connect(`ws://${robotIP}`)
  }

  disconnect() {
    this.ros.close()
  }

  listen(options: TopicOptions) {
    this.topicManager.listen(options)
  }

  subscribe(options: TopicOptions, handler: Function) {
    this.topicManager.subscribe(options, handler)
  }

  publish(options: TopicOptions, payload: any) {
    this.topicManager.publish(options, payload)
  }

  callService(options: ServiceOptions, payload: any) {
    return this.serviceManager.callService(options, payload)
  }

  setListeners(onConnection: Function, onClose: Function, onError: Function) {
    this.ros.on('connection', this.onConnection(onConnection))
    this.ros.on('close', this.onClose(onClose))
    this.ros.on('error', this.onError(onError))
  }

  private onConnection(onConnection: Function): (event: any) => void {
    return () => {
      this.topicManager.reconnectAllDisconnectedHandler()
      onConnection
    }
  }

  private onClose(onClose: Function): (event: any) => void {
    return () => {
      this.topicManager.unsubscribeAllTopics()
      onClose()
    }
  }

  private onError(onError: Function): (event: any) => void {
    return error => {
      if (process.env.NODE_ENV !== 'production') {
        console.error('RosError', error)
      }

      onError(error)

      if (this.shouldTryToReconnect) {
        this.connect(this.robotIP)
      }
    }
  }
}

export default RosClient
