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
  public gamepad: Gamepad
  private mapping: GamepadMapping

  constructor(gamepad: Gamepad) {
    this.gamepad = gamepad
    this.mapping = this.detectMapping(gamepad)
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
