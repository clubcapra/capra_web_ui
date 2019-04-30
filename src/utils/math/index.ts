import { Twist } from './types'

export const mapGamepadToTwist = (
  x: number,
  z: number,
  factor: number
): Twist => {
  return {
    linear: {
      x: x * factor,
      y: 0,
      z: 0,
    },
    angular: {
      x: 0,
      y: 0,
      z: z * factor,
    },
  }
}
