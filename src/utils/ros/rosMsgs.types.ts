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
