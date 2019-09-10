import {
  GamepadBtn,
  Dpad,
  Stick,
  StickAxis,
  GamepadData,
} from 'utils/gamepad/@types'
import mappings from './mappings'
import { GamepadMapping } from 'utils/gamepad/mappings/types'

export function isSupported(): boolean {
  if ('getGamepads' in navigator) return true
  if ('onconnectedgamepad' in window) return true
  return false
}

const getButton = (gamepad: Gamepad, button: number): GamepadButton => {
  return gamepad.buttons[button]
}

const getAxis = (gamepad: Gamepad, axis: number): number => {
  return gamepad.axes[axis]
}

export const getButtonPressed = (button: GamepadBtn | Dpad) => ({
  mapping,
  gamepad,
}: GamepadData): boolean => {
  const index = mapping.buttons[button]
  const nativeButton = getButton(gamepad, index)

  return typeof nativeButton == 'number'
    ? nativeButton > 0.1
    : nativeButton.pressed
}

export const getButtonValue = (button: GamepadBtn | Dpad) => ({
  mapping,
  gamepad,
}: GamepadData): number => {
  const index = mapping.buttons[button]
  const nativeButton = getButton(gamepad, index)

  return typeof nativeButton == 'number' ? nativeButton : nativeButton.value
}

export const getStick = (stick: Stick) => ({
  mapping,
  gamepad,
}: GamepadData): StickAxis => {
  const stickMapping = mapping.sticks[stick]

  const horizontal = getAxis(gamepad, stickMapping.horizontal)
  const vertical = getAxis(gamepad, stickMapping.vertical)

  return {
    horizontal: stickMapping.isRightPositive ? horizontal : -horizontal,
    vertical: stickMapping.isUpPositive ? vertical : -vertical,
  }
}

export const getButtonPressedFromPrev = (button: GamepadBtn | Dpad) => (
  data: GamepadData
): boolean => {
  return getButtonPressed(button)(data)
}

export const getTogglePressed = (button: GamepadBtn | Dpad) => (
  data: GamepadData
): boolean =>
  getButtonPressed(button)(data) && !getButtonPressedFromPrev(button)(data)

const defaultMapping = mappings[0]

/**
 * Detects the mapping of the gamepad based on the id given by the browser.
 * If no specific mapping is found it will default to the mapping defined
 * by the Gamepad Api standard: https://www.w3.org/TR/gamepad/#remapping
 * @param gamepad the raw gamepad that needs a mapping
 */
export const detectMapping = (gamepad: Gamepad): GamepadMapping => {
  //TODO support different mappings
  return defaultMapping
}

export const isSpaceMouse = ({ gamepad }: GamepadData): boolean =>
  gamepad.id.includes('SpaceMouse')
