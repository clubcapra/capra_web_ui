// import { Module, VuexModule, Mutation, Action } from 'vuex-module-decorators'
import { VuexModule, mutation, action, getter, Module } from "vuex-class-component";

@Module({ namespacedPath: 'ros/'})
export default class RosModule extends VuexModule {
  connected = false
  robotIP = 'localhost:9090'

  @mutation
  setConnected(isConnected: boolean) {
    this.connected = isConnected
  }

  @mutation
  setRobotIP(ip: string) {
    this.robotIP = ip
  }

  @action
  async onConnect() {
    this.setConnected(true)
  }

  @action
  async onDisconnect() {
    this.setConnected(false)
  }
}
