import mappings from './mappings'
import {
  GamepadMapping,
  NativeGamepad,
  ButtonNames,
  AxisNames,
  StickDirections,
  Button,
  Axis,
  AxisButton,
} from './mappings/types'

export default class CustomGamepad {
  private readonly axisButtonRange = 0.75

  private gamepad: NativeGamepad
  private mapping: GamepadMapping

  constructor(gamepad: NativeGamepad) {
    this.gamepad = gamepad
    this.mapping = this.detectMapping(gamepad.id, navigator.userAgent)
  }

  getButtonPressed = (name: ButtonNames | StickDirections) => {
    const button = this.mapping.buttons[name as ButtonNames]
    if (button) {
      return this.getNativeButton(button).pressed
    }

    const axisButton = this.mapping.axisButtons[name as StickDirections]
    if (axisButton) {
      const axis = this.getNativeAxis(axisButton)
      return axis < -this.axisButtonRange || axis > this.axisButtonRange
    }

    return false
  }

  getButtonValue = (name: ButtonNames): number => {
    const button = this.mapping.buttons[name]
    return button ? this.getNativeButton(button).value : 0
  }

  getAxis = (name: AxisNames) => {
    const axis = this.mapping.axes[name]
    return axis ? this.getNativeAxis(axis) : 0
  }

  private getNativeAxis(axis: Axis | AxisButton) {
    return this.gamepad.axes[axis.index]
  }

  private getNativeButton(button: Button) {
    return this.gamepad.buttons[button.index]
  }

  private detectMapping(id: string, browser: string) {
    const compatibleMappings = mappings.filter(mapping =>
      this.isCompatible(mapping, id, browser)
    )

    return compatibleMappings.length > 0 ? compatibleMappings[0] : mappings[0]
  }

  private isCompatible = (
    mapping: GamepadMapping,
    id: string,
    browser: string
  ) =>
    mapping.supported.some(
      supported =>
        id.includes(supported.id) &&
        browser.includes(supported.os) &&
        browser.includes(browser)
    )
}
