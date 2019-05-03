import mappings from './mappings'
import {
  GamepadMapping,
  Stick,
  GamepadBtn,
  Dpad,
  StickAxis,
} from './mappings/types'

export default class CustomGamepad {
  private readonly AXIS_PRECISION = 3
  private gamepad: Gamepad
  private mapping: GamepadMapping

  constructor(gamepad: Gamepad) {
    this.gamepad = gamepad
    this.mapping = this.detectMapping(gamepad)
  }

  getButtonPressed = (button: GamepadBtn | Dpad): boolean => {
    const index = this.mapping.buttons[button]
    const nativeButton = this.getNativeButton(index)
    return typeof nativeButton == 'number'
      ? nativeButton > 0.1
      : nativeButton.pressed
  }

  getButtonValue = (button: GamepadBtn | Dpad): number => {
    const index = this.mapping.buttons[button]
    const nativeButton = this.getNativeButton(index)
    return typeof nativeButton == 'number' ? nativeButton : nativeButton.value
  }

  getStick = (stick: Stick): StickAxis => {
    const stickMapping = this.mapping.sticks[stick]
    return {
      horizontal: this.getNativeAxis(stickMapping.horizontal),
      vertical: this.getNativeAxis(stickMapping.vertical),
    }
  }

  private getNativeAxis(axis: number) {
    return this.gamepad.axes[axis]
  }

  private getNativeButton(button: number) {
    return this.gamepad.buttons[button]
  }

  private detectMapping(gamepad: Gamepad) {
    return mappings[0]
  }
}
