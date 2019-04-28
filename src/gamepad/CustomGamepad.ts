import mappings from './mappings'
import { GamepadMapping } from './mappings/types'

type NativeGamepad = Gamepad

export default class CustomGamepad {
  gamepad: NativeGamepad
  mapping: GamepadMapping

  private readonly axisButtonRange = 0.75

  constructor(gamepad: NativeGamepad) {
    this.gamepad = gamepad
    this.mapping = this.detectMapping(gamepad.id, navigator.userAgent)
  }

  detectMapping(id: string, browser: string) {
    const compatibleMappings = mappings.filter(mapping =>
      this.isCompatible(mapping, id, browser)
    )

    return compatibleMappings.length > 0 ? compatibleMappings[0] : mappings[0]
  }

  isCompatible = (mapping: GamepadMapping, id: string, browser: string) =>
    mapping.supported.some(
      supported =>
        id.includes(supported.id) &&
        browser.includes(supported.os) &&
        browser.includes(browser)
    )

  getButtonPressed = (name: string) => {
    const { axes, buttons } = this.gamepad

    const button = this.mapping.buttons[name]
    if (button) {
      return buttons[button.index].pressed
    }

    const axisButton = this.mapping.axesButtons[name]
    if (axisButton) {
      const axis = axes[axisButton.axis]
      return axisButton.direction < 0
        ? axis < -this.axisButtonRange
        : axis > this.axisButtonRange
    }

    return false
  }

  getAxis = (name: string) => {
    const axis = this.mapping.axes[name]
    return axis ? this.gamepad.axes[axis.index] : 0
  }
}
