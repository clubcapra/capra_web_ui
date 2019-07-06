import { VuexModule, Module, mutation } from 'vuex-class-component'

@Module({ namespacedPath: 'gamepad/' })
export default class GamepadModule extends VuexModule {
  isArmControlled = false

  @mutation
  toggleIsArmControlled() {
    this.isArmControlled = !this.isArmControlled
  }

  @mutation
  setIsArmControlled(value: boolean) {
    this.isArmControlled = value
  }
}
