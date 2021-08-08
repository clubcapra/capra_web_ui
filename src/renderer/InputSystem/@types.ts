type InputType =
  | 'keyboard'
  | 'gamepad'
  | 'gamepadBtn'
  | 'gamepadAxis'
  | 'spacemouse'
  | 'none'

interface IBinding {
  type: InputType
}

interface KeyboardBinding extends IBinding {
  type: 'keyboard'
  code: string
  onKeyDown?: boolean
}

interface GamepadBinding extends IBinding {
  type: 'gamepad'
}

interface GamepadBtnBinding extends IBinding {
  type: 'gamepadBtn'
  button: number
}

interface GamepadAxisBinding extends IBinding {
  type: 'gamepadAxis'
  axis: number
}

interface SpacemouseBinding extends IBinding {
  type: 'spacemouse'
}

export type Binding =
  | KeyboardBinding
  | GamepadBinding
  | GamepadBtnBinding
  | GamepadAxisBinding
  | SpacemouseBinding

interface IContext {
  type: InputType
}

interface KeyboardContext extends IContext {
  type: 'keyboard'
}

export interface GamepadContext extends IContext {
  type: 'gamepad' | 'gamepadBtn' | 'gamepadAxis' | 'spacemouse'
  id: string
  gamepadState: GamepadState
}

export interface GamepadBtnContext extends GamepadContext {
  type: 'gamepadBtn'
}

export interface GamepadAxisContext extends GamepadContext {
  type: 'gamepadAxis'
  value: number
}

export interface SpaceMouseContext extends GamepadContext {
  type: 'spacemouse'
  axes: number[]
  buttons: GamepadButton[]
}

export interface EmptyContext extends IContext {
  type: 'none'
}

export type Context =
  | KeyboardContext
  | GamepadContext
  | GamepadBtnContext
  | GamepadAxisContext
  | SpaceMouseContext
  | EmptyContext

export type Action = {
  name: string
  bindings: Readonly<Binding[]>
  perform: (context: Context) => void
}

export interface GamepadState {
  gamepad: Gamepad
  prevGamepad: Gamepad
  index: number
}
