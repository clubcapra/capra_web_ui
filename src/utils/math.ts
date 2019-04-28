type Vector2 = { x: Number; y: Number }
type Vector3 = { x: Number; y: Number; z: Number }

type cmd_vel = { linear: Vector3, angular: Vector3 }

export const mapGamepadToTwist = (xAxis: number, zAxis: number, factor: number): cmd_vel => {
  const cmd_vel: cmd_vel = {
    linear: {
      x: xAxis * factor, y: 0, z: 0
    },
    angular: {
      x: 0, y: 0, z: zAxis * factor
    }
  }

  return cmd_vel
}
