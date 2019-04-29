import {
  Module,
  VuexModule,
  Mutation,
  Action,
  getModule,
} from 'vuex-module-decorators'
import store from '@/store'

@Module({ dynamic: true, store, name: 'ros', namespaced: true })
class RosModule extends VuexModule {
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

export default getModule(RosModule)
