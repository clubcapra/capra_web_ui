import { getButtonValue, getButtonPressed, getStick } from './GamepadUtils'
import mappings from './mappings'
import { GamepadMapping } from './mappings/types'
import { GamepadBtn, Dpad, Stick, StickAxis } from 'utils/gamepad/@types'

export default class CustomGamepad {
  gamepad: Gamepad
  private mapping: GamepadMapping
  private prevGamepad: Gamepad

  constructor(gamepad: Gamepad, prevGamepad: Gamepad) {
    this.gamepad = gamepad
    this.prevGamepad = prevGamepad
    this.mapping = this.detectMapping(gamepad)
  }

  getButtonPressed = (button: GamepadBtn | Dpad): boolean => {
    return getButtonPressed(this.gamepad, this.mapping, button)
  }

  getButtonPressedFromPrev = (button: GamepadBtn | Dpad): boolean => {
    return getButtonPressed(this.prevGamepad, this.mapping, button)
  }

  getTogglePressed = (button: GamepadBtn | Dpad): boolean => {
    return (
      this.getButtonPressed(button) && !this.getButtonPressedFromPrev(button)
    )
  }

  getButtonValue = (button: GamepadBtn | Dpad): number => {
    return getButtonValue(this.gamepad, this.mapping, button)
  }

  getStick = (stick: Stick): StickAxis => {
    return getStick(this.gamepad, this.mapping, stick)
  }

  // TODO: actually support multpile mappings
  private detectMapping = (gamepad: Gamepad) => {
    return mappings[0]
  }
}
