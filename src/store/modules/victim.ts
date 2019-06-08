import { VuexModule, Module, mutation } from 'vuex-class-component'

@Module({ namespacedPath: 'victim/' })
export default class VictimModule extends VuexModule {
  cameraMotion = 'camera3d_rgb'
  cameraThermal = 'camera3d_rgb'
  cameraQr = 'camera3d_rgb'
  cameraHazmat = 'camera3d_rgb'

  @mutation
  setCameraMotion(cameraName: string) {
    this.cameraMotion = cameraName
  }
  @mutation
  setCameraThermal(cameraName: string) {
    this.cameraThermal = cameraName
  }
  @mutation
  setCameraQr(cameraName: string) {
    this.cameraQr = cameraName
  }
  @mutation
  setCameraHazmat(cameraName: string) {
    this.cameraHazmat = cameraName
  }
}
