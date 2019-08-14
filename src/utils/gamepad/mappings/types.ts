import { Stick, StickAxisMapping, GamepadBtn, Dpad } from 'utils/gamepad/@types'

export interface GamepadMapping {
  sticks: StickMapping
  buttons: ButtonMapping
}

type StickMapping = { [key in Stick]: StickAxisMapping }

type ButtonMapping = { [key in GamepadBtn | Dpad]: number }
