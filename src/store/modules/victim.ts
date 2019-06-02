import { VuexModule, Module, mutation } from 'vuex-class-component'

@Module({ namespacedPath: 'victim/' })
export default class VictimModule extends VuexModule {
  cameraMotion = 'camera1'
  cameraThermal = 'camera1'
  cameraQr = 'camera1'
  cameraHazmat = 'camera1'

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
