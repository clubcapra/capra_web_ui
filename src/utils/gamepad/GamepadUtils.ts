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

const getRawButton = (
  button: GamepadBtn | Dpad,
  gamepad: Gamepad,
  mapping: GamepadMapping
) => {
  const rawBtnIndex = mapping.buttons[button]
  return gamepad.buttons[rawBtnIndex]
}

const getRawAxis = (gamepad: Gamepad) => (axis: number): number =>
  gamepad.axes[axis]

export const getButtonPressed = (gamepad: Gamepad, mapping: GamepadMapping) => (
  button: GamepadBtn | Dpad
): boolean => {
  const rawBtn = getRawButton(button, gamepad, mapping)
  return typeof rawBtn == 'number' ? rawBtn > 0.1 : rawBtn.pressed
}

export const getButtonValue = ({ gamepad, mapping }: GamepadData) => (
  button: GamepadBtn | Dpad
): number => {
  const rawBtn = getRawButton(button, gamepad, mapping)
  return typeof rawBtn == 'number' ? rawBtn : rawBtn.value
}

export const getStick = ({ mapping, gamepad }: GamepadData) => (
  stick: Stick
): StickAxis => {
  const rawGamepadAxis = getRawAxis(gamepad)
  const stickMapping = mapping.sticks[stick]
  const horizontal = rawGamepadAxis(stickMapping.horizontal)
  const vertical = rawGamepadAxis(stickMapping.vertical)

  return {
    horizontal: stickMapping.isRightPositive ? horizontal : -horizontal,
    vertical: stickMapping.isUpPositive ? vertical : -vertical,
  }
}

export const getButtonDown = ({
  gamepad,
  prevGamepad,
  mapping,
}: GamepadData) => (button: GamepadBtn | Dpad): boolean => {
  if (getButtonPressed(gamepad, mapping)(button) && !prevGamepad) return true

  return (
    getButtonPressed(gamepad, mapping)(button) &&
    !getButtonPressed(prevGamepad, mapping)(button)
  )
}

export const getButtonUp = ({ gamepad, prevGamepad, mapping }: GamepadData) => (
  button: GamepadBtn | Dpad
): boolean => {
  if (!prevGamepad) return false

  return (
    !getButtonPressed(gamepad, mapping)(button) &&
    getButtonPressed(prevGamepad, mapping)(button)
  )
}

/**
 * Detects the mapping of the gamepad based on the id given by the browser.
 * If no specific mapping is found it will default to the mapping defined
 * by the Gamepad Api standard: https://www.w3.org/TR/gamepad/#remapping
 * @param gamepad the raw gamepad that needs a mapping
 */
export const detectMapping = (gamepad: Gamepad): GamepadMapping => {
  //TODO support different mappings
  return mappings[0]
}

export const isSpaceMouse = ({ gamepad: { id } }: GamepadData): boolean =>
  id.includes('SpaceMouse')

export const getDataFunctions = (data: GamepadData) => ({
  isButtonDown: getButtonDown(data),
  isButtonUp: getButtonUp(data),
  isButtonPressed: getButtonPressed(data.gamepad, data.mapping),
  getStick: getStick(data),
  getButtonValue: getButtonValue(data),
})
