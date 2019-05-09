import { Module, VuexModule, getModule } from 'vuex-module-decorators'

@Module({ name: 'teleop', namespaced: true })
export default class TeleopModule extends VuexModule {
  leftCamera = 'camera1'
  rightCamera = 'camera2'
}
