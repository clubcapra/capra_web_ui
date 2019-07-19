import { getButtonValue, getButtonPressed, getStick } from './GamepadUtils'
import mappings from './mappings'
import {
  GamepadMapping,
  Stick,
  GamepadBtn,
  Dpad,
  StickAxis,
} from './mappings/types'

export default class CustomGamepad {
  private mapping: GamepadMapping

  constructor(gamepad: Gamepad) {
    this.gamepad = gamepad
    this.mapping = this.detectMapping(gamepad)
  }

  set gamepad(newGamepad: Gamepad) {
    this.gamepad = newGamepad
    if (this.gamepad.id !== newGamepad.id)
      this.mapping = this.detectMapping(newGamepad)
  }

  get isSpaceMouse() {
    const gamepadName = this.gamepad.id
    return (
      gamepadName.includes('spacenavigator') ||
      gamepadName.includes('space navigator')
    )
  }

  getButtonPressed(button: GamepadBtn | Dpad): boolean {
    return getButtonPressed(this.gamepad, this.mapping, button)
  }

  getButtonValue(button: GamepadBtn | Dpad): number {
    return getButtonValue(this.gamepad, this.mapping, button)
  }

  getStick(stick: Stick): StickAxis {
    return getStick(this.gamepad, this.mapping, stick)
  }

  // TODO: actually support multpile mappings
  private detectMapping(gamepad: Gamepad) {
    return mappings[0]
  }
}
