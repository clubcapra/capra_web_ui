import { Twist } from './types'

export const mapGamepadToTwist = (
  xAxis: number,
  zAxis: number,
  factor: number
) => {
  const cmd_vel: Twist = {
    linear: {
      x: xAxis * factor,
      y: 0,
      z: 0,
    },
    angular: {
      x: 0,
      y: 0,
      z: zAxis * factor,
    },
  }

  return cmd_vel
}
