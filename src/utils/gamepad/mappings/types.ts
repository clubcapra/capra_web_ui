export interface GamepadMapping {
  name: string
  supported: Array<{ browser: string; id: string; os: string }>
  axes: Record<AxisNames, Axis>
  buttons: Record<ButtonNames, Button>
  axisButtons: Record<StickDirections, AxisButton>
}

export type NativeGamepad = Gamepad

export type ButtonNames = buttons | dpad | shoulder | trigger | stick
export type AxisNames = leftStickAxis | rightStickAxis | dpadAxis | trigger
export type StickDirections = rightStickDirections | leftStickDirections

type buttons = 'a' | 'b' | 'x' | 'y' | 'start' | 'back' | 'home'

type shoulder = 'left shoulder' | 'right shoulder'

type trigger = 'left trigger' | 'right trigger'

type dpad = 'dpad left' | 'dpad right' | 'dpad up' | 'dpad down'
type dpadAxis = 'dpad x' | 'dpad y'

type stick = 'left stick' | 'right stick'
type leftStickAxis = 'left stick x' | 'left stick y'
type rightStickAxis = 'right stick x' | 'right stick y'
type leftStickDirections =
  | 'left stick up'
  | 'left stick down'
  | 'left stick left'
  | 'left stick right'
type rightStickDirections =
  | 'right stick up'
  | 'right stick down'
  | 'right stick left'
  | 'right stick right'

export interface Button {
  index: number
}

export interface Axis {
  index: number
}

export interface AxisButton extends Button {
  direction: number
}
