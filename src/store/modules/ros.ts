import { VuexModule, mutation, action, Module } from 'vuex-class-component'
import RosClient from '@/utils/ros/RosClient'

@Module({ namespacedPath: 'ros/' })
export default class RosModule extends VuexModule {
  connected = false
  connecting = false
  robotIP = 'localhost'
  port = '9090'

  get url() {
    return `${this.robotIP}:${this.port}`
  }

  @mutation
  setConnected(isConnected: boolean) {
    this.connected = isConnected
  }

  @mutation
  setConnecting(isConnecting: boolean) {
    this.connecting = isConnecting
  }

  @mutation
  setRobotIP(ip: string) {
    this.robotIP = ip
  }

  @mutation
  setPort(port: string) {
    this.port = port
  }

  @mutation
  onConnecting() {
    this.connecting = true
    this.connected = false
  }

  @action
  async onConnect() {
    this.setConnecting(false)
    this.setConnected(true)
  }

  @action
  async onDisconnect() {
    this.setConnecting(false)
    this.setConnected(false)
  }

  @action
  async connect() {
    this.onConnecting()
    RosClient.connect(this.robotIP, this.port)
  }

  //TODO maybe handle subscribe/unsubscribe
}
