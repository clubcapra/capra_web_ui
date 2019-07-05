export interface IVector2 {
  x: number
  y: number
}

export interface IVector3 {
  x: number
  y: number
  z: number
}

export class Vector3 implements IVector3, IVector2 {
  constructor(
    public x: number = 0,
    public y: number = 0,
    public z: number = 0
  ) {}

  static zero(): Vector3 {
    return new Vector3()
  }

  static one(): Vector3 {
    return new Vector3(1, 1, 1)
  }
}

export interface ITwist {
  linear: Vector3
  angular: Vector3
}

export class Twist implements ITwist {
  constructor(public linear: Vector3, public angular: Vector3) {}
}
