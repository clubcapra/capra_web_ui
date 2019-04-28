export interface GamepadMapping {
  name: string
  axes: Record<string, Axis>
  buttons: Record<string, Button>
  axesButtons: Record<string, AxisButton>
  supported: Array<{ browser: string; id: string; os: string }>
}

interface Button {
  index: number
}

interface Axis {
  index: number
}

interface AxisButton {
  axis: number
  direction: number
}
