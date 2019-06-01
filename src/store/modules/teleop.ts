import { VuexModule, Module, mutation } from 'vuex-class-component'

@Module({ namespacedPath: 'teleop/' })
export default class TeleopModule extends VuexModule {
  leftCamera = 'camera1'
  rightCamera = 'camera2'

  @mutation
  setLeftCamera(cameraName: string) {
    this.leftCamera = cameraName
  }

  @mutation
  setRightCamera(cameraName: string) {
    this.rightCamera = cameraName
  }
}
