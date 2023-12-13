import { Vector3 } from '@/renderer/utils/math/types';
export interface IJoyMsg {
  header: {
    stamp: {
      sec: number;
      nsecs: number;
    };
    frame_id: string;
  };
  axes: readonly number[];
  buttons: readonly number[];
}

export interface ITwistMsg {
  linear: Vector3;
  angular: Vector3;
}
