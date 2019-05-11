import { VuexModule, Module } from 'vuex-class-component'

@Module({ namespacedPath: 'teleop/' })
export default class TeleopModule extends VuexModule {
  leftCamera = 'camera1'
  rightCamera = 'camera2'
}
