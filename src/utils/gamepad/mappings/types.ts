import { Stick, GamepadBtn, Dpad } from 'utils/gamepad/@types'

export interface GamepadMapping {
  sticks: StickMapping
  buttons: ButtonMapping
}

interface StickAxisMapping {
  horizontal: number
  isUpPositive: boolean
  vertical: number
  isRightPositive: boolean
}

type StickMapping = { [key in Stick]: StickAxisMapping }

type ButtonMapping = { [key in GamepadBtn | Dpad]: number }
