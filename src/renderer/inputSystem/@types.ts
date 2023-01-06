type InputType =
  | 'keyboard'
  | 'gamepad'
  | 'gamepadBtnDown'
  | 'gamepadBtnUp'
  | 'gamepadAxis'
  | 'spacemouse'
  | 'none';

interface IBinding {
  type: InputType;
}

interface KeyboardBinding extends IBinding {
  type: 'keyboard';
  code: string;
  onKeyDown?: boolean;
}

interface GamepadBinding extends IBinding {
  type: 'gamepad';
}

export interface GamepadButtonBinding extends IBinding {
  type: 'gamepadBtnDown' | 'gamepadBtnUp';
  button: number;
}

interface GamepadBtnDownBinding extends GamepadButtonBinding {
  type: 'gamepadBtnDown';
}

interface GamepadBtnUpBinding extends GamepadButtonBinding {
  type: 'gamepadBtnUp';
}

interface GamepadAxisBinding extends IBinding {
  type: 'gamepadAxis';
  axis: number;
}

interface SpacemouseBinding extends IBinding {
  type: 'spacemouse';
}

export type Binding =
  | KeyboardBinding
  | GamepadBinding
  | GamepadBtnDownBinding
  | GamepadBtnUpBinding
  | GamepadAxisBinding
  | SpacemouseBinding;

interface IContext {
  type: InputType;
}

interface KeyboardContext extends IContext {
  type: 'keyboard';
}

export interface GamepadContext extends IContext {
  type:
    | 'gamepad'
    | 'gamepadBtnDown'
    | 'gamepadBtnUp'
    | 'gamepadAxis'
    | 'spacemouse';
  id: string;
  gamepadState: GamepadState;
}

export interface GamepadBtnDownContext extends GamepadContext {
  type: 'gamepadBtnDown';
}

export interface GamepadBtnUpContext extends GamepadContext {
  type: 'gamepadBtnUp';
}

export interface GamepadAxisContext extends GamepadContext {
  type: 'gamepadAxis';
  value: number;
}

export interface SpaceMouseContext extends GamepadContext {
  type: 'spacemouse';
  axes: number[];
  buttons: GamepadButton[];
}

export interface EmptyContext extends IContext {
  type: 'none';
}

export type Context =
  | KeyboardContext
  | GamepadContext
  | GamepadBtnDownContext
  | GamepadAxisContext
  | SpaceMouseContext
  | EmptyContext;

export type Action = {
  name: string;
  bindings: Readonly<Binding[]>;
  perform: (context: Context) => void;
};

export interface GamepadState {
  gamepad: Gamepad;
  lastGamepad: Gamepad;
  index: number;
}
