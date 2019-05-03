import { Module, VuexModule, getModule } from 'vuex-module-decorators'
import store from '@/store'

@Module({ dynamic: true, store, name: 'teleop', namespaced: true })
class TeleopModule extends VuexModule {
  leftCamera = 'camera1'
  rightCamera = 'camera2'
}

export default getModule(TeleopModule)
