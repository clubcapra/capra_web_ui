import {
  Stick,
  GamepadBtn,
  GamepadMapping,
  Dpad,
  StickAxis,
} from './mappings/types'

const getButton = (gamepad: Gamepad, button: number) => {
  return gamepad.buttons[button]
}

const getAxis = (gamepad: Gamepad, axis: number) => {
  return gamepad.axes[axis]
}

export const getButtonPressed = (
  gamepad: Gamepad,
  mapping: GamepadMapping,
  button: GamepadBtn | Dpad
): boolean => {
  const index = mapping.buttons[button]
  const nativeButton = getButton(gamepad, index)

  return typeof nativeButton == 'number'
    ? nativeButton > 0.1
    : nativeButton.pressed
}

export const getButtonValue = (
  gamepad: Gamepad,
  mapping: GamepadMapping,
  button: GamepadBtn | Dpad
): number => {
  const index = mapping.buttons[button]
  const nativeButton = getButton(gamepad, index)

  return typeof nativeButton == 'number' ? nativeButton : nativeButton.value
}

export const getStick = (
  gamepad: Gamepad,
  mapping: GamepadMapping,
  stick: Stick
): StickAxis => {
  const stickMapping = mapping.sticks[stick]

  const horizontal = getAxis(gamepad, stickMapping.horizontal)
  const vertical = getAxis(gamepad, stickMapping.vertical)

  return {
    horizontal: stickMapping.isRightPositive ? horizontal : -horizontal,
    vertical: stickMapping.isUpPositive ? vertical : -vertical,
  }
}
