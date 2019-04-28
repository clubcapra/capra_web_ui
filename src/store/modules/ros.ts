import { Module, VuexModule, Mutation, Action } from 'vuex-module-decorators'
import store from '..'

@Module({ dynamic: true, store, name: 'ros', namespaced: true })
export default class RosModule extends VuexModule {
  connected = false
  robotIP = 'localhost:9090'

  @Mutation
  setConnected(isConnected: boolean) {
    this.connected = isConnected
  }

  @Mutation
  setRobotIP(ip: string) {
    this.robotIP = ip
  }

  @Action({ commit: 'setConnected' })
  onConnect() {
    return true
  }

  @Action({ commit: 'setConnected' })
  onDisconnect() {
    return false
  }
}
