export interface GamepadMapping {
  sticks: StickMapping
  buttons: ButtonMapping
}

type StickMapping = { [key in Stick]: StickAxisMapping }

type ButtonMapping = { [key in GamepadBtn | Dpad]: number }

export enum GamepadBtn {
  A = 'A',
  B = 'B',
  X = 'X',
  Y = 'Y',
  LB = 'LB',
  RB = 'RB',
  LT = 'LT',
  RT = 'RT',
  Back = 'Back',
  Start = 'Start',
  LS = 'LS',
  RS = 'RS',
}

export enum Dpad {
  Up = 'DpadUp',
  Down = 'DpadDown',
  Left = 'DpadLeft',
  Right = 'DpadRight',
}

export enum Stick {
  Left = 'leftStick',
  Right = 'rightStick',
}

export interface StickDirection {
  Up: number
  Down: number
  Left: number
  Right: number
}

export interface StickAxis {
  [key: string]: number
  horizontal: number
  vertical: number
}

export interface StickAxisMapping {
  horizontal: number
  isUpPositive: boolean
  vertical: number
  isRightPositive: boolean
}
