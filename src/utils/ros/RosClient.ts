import { Ros } from 'roslib'
import { TopicOptions, ServiceOptions } from './types'
import TopicManager from './TopicManager'
import ServiceManager from './ServiceManager'
import { rosModule } from '@/store'

class RosClient {
  ros: Ros = new Ros({})
  private topicManager: TopicManager = new TopicManager(this.ros)
  private serviceManager: ServiceManager = new ServiceManager(this.ros)
  private robotIP?: string
  private port?: string
  private shouldTryToReconnect: boolean = false
  private connected: boolean = false

  connect(robotIP = 'localhost', port = '9090', shouldTryToReconnect = false) {
    this.shouldTryToReconnect = shouldTryToReconnect
    this.robotIP = robotIP
    this.port = port
    const url = `ws://${robotIP}:${port}`

    // if (this.connected) {
    //   this.ros.close()
    //   this.ros = new Ros({})
    //   this.initListeners()
    // }

    rosModule.onConnecting()
    this.ros.connect(url)
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

  unsubscribe(options: TopicOptions) {
    this.topicManager.unsubscribe(options)
  }

  publish(options: TopicOptions, payload: any) {
    this.topicManager.publish(options, payload)
  }

  callService(options: ServiceOptions, payload?: any) {
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
      this.connected = true
      onConnection()
    }
  }

  private onClose(onClose: Function): (event: any) => void {
    return () => {
      this.topicManager.unsubscribeAllTopics()
      this.connected = false
      onClose()
    }
  }

  private onError(onError: Function): (event: any) => void {
    return error => {
      this.connected = false
      if (process.env.NODE_ENV !== 'production') {
        console.error('RosError', error)
      }

      onError(error)

      if (this.shouldTryToReconnect) {
        this.connect(this.robotIP, this.port, true)
      }
    }
  }
}

export default new RosClient()
