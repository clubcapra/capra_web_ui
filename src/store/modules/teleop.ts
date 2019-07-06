import { VuexModule, Module, mutation } from 'vuex-class-component'

@Module({ namespacedPath: 'teleop/' })
export default class TeleopModule extends VuexModule {
  leftCamera = 'camera3d_rgb'
  rightCamera = 'camera_back'
  bottomCamera = 'camera_arm'

  @mutation
  setLeftCamera(cameraName: string) {
    this.leftCamera = cameraName
  }

  @mutation
  setRightCamera(cameraName: string) {
    this.rightCamera = cameraName
  }

  @mutation
  setBottomCamera(cameraName: string) {
    this.bottomCamera = cameraName
  }
}
