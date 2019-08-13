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
  x: number = 0
  y: number = 0
  z: number = 0

  constructor(x: number = 0, y: number = 0, z: number = 0) {
    this.x = x
    this.y = y
    this.z = z
  }

  static zero(): Vector3 {
    return new Vector3()
  }

  static one(): Vector3 {
    return new Vector3(1, 1, 1)
  }
}
