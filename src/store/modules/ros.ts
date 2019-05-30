import { VuexModule, mutation, action, Module } from 'vuex-class-component'
import RosClient from '@/utils/ros/RosClient'

@Module({ namespacedPath: 'ros/' })
export default class RosModule extends VuexModule {
  connected = false
  connecting = false
  robotIP = 'localhost'
  port = '9090'

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

  @action
  async onConnecting() {
    this.setConnecting(true)
    this.setConnected(false)
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
}
