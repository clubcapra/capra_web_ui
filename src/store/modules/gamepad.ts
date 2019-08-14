import { VuexModule, Module, mutation } from 'vuex-class-component'

@Module({ namespacedPath: 'gamepad/' })
export default class GamepadModule extends VuexModule {
  isArmControlled = false
  spaceMouseIndex = 0
  currentGamepadIndex = 0

  @mutation
  toggleIsArmControlled() {
    this.isArmControlled = !this.isArmControlled
  }

  @mutation
  setIsArmControlled(value: boolean) {
    this.isArmControlled = value
  }

  @mutataion
  setSpaceMouseIndex(value: int){
    this.spaceMouseIndex = value
  }

  @mutataion
  setCurrentGamepadIndex(value: int){
    this.currentGamepadIndex = value
  }
}
