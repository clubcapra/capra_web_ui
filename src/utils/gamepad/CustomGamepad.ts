import mappings from './mappings'
import {
  GamepadMapping,
  Stick,
  GamepadBtn,
  Dpad,
  StickAxis,
} from './mappings/types'

export default class CustomGamepad {
  public gamepad: Gamepad
  private mapping: GamepadMapping

  constructor(gamepad: Gamepad) {
    this.gamepad = gamepad
    this.mapping = this.detectMapping(gamepad)
  }

  getButtonPressed(button: GamepadBtn | Dpad): boolean {
    const index = this.mapping.buttons[button]
    const nativeButton = this.getButton(index)
    return typeof nativeButton == 'number'
      ? nativeButton > 0.1
      : nativeButton.pressed
  }

  getButtonValue(button: GamepadBtn | Dpad): number {
    const index = this.mapping.buttons[button]
    const nativeButton = this.getButton(index)

    return typeof nativeButton == 'number' ? nativeButton : nativeButton.value
  }

  getStick(stick: Stick): StickAxis {
    const stickMapping = this.mapping.sticks[stick]

    const horizontal = this.getAxis(stickMapping.horizontal)
    const vertical = this.getAxis(stickMapping.vertical)

    return {
      horizontal: stickMapping.isRightPositive ? horizontal : -horizontal,
      vertical: stickMapping.isUpPositive ? vertical : -vertical,
    }
  }

  private getAxis(axis: number) {
    return this.gamepad.axes[axis]
  }

  private getButton(button: number) {
    return this.gamepad.buttons[button]
  }

  private detectMapping(_gamepad: Gamepad) {
    return mappings[0]
  }
}
