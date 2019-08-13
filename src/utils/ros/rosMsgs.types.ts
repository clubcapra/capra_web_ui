import { Vector3 } from 'utils/math/types'
export interface IJoyMsg {
  header: {
    seq: number
    stamp: {
      sec: number
      nsecs: number
    }
    frame_id: string
  }
  axes: number[]
  buttons: number[]
}

export interface ITwistMsg {
  linear: Vector3
  angular: Vector3
}

export class Twist implements ITwistMsg {
  linear: Vector3
  angular: Vector3

  constructor(linear: Vector3, angular: Vector3) {
    this.linear = linear
    this.angular = angular
  }
}
