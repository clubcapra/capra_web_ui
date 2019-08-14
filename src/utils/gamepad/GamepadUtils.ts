import { GamepadMapping } from './mappings/types'
import { GamepadBtn, Dpad, Stick, StickAxis } from 'utils/gamepad/@types'
import CustomGamepad from 'utils/gamepad/CustomGamepad'

const getButton = (gamepad: Gamepad, button: number): GamepadButton => {
  return gamepad.buttons[button]
}

const getAxis = (gamepad: Gamepad, axis: number): number => {
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

export const isSpaceMouse = (gamepad: CustomGamepad): boolean =>
  gamepad.gamepad.id.includes('SpaceMouse')
