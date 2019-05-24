import { VuexModule, Module, mutation } from 'vuex-class-component'

@Module({ namespacedPath: 'victim/' })
export default class VictimModule extends VuexModule {
  camera = 'camera1'

  @mutation
  setCamera(cameraName: string) {
    this.camera = cameraName
  }
}
