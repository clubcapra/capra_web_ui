// Type definitions for ros3d
// Definitions by: Charles Giguere <https://github.com/IceSentry>
// Definitions: https://github.com/DefinitelyTyped/DefinitelyTyped

declare module 'ros2d' {
  import { Ros } from 'roslib'

  type ViewerScene = unknown

  class Viewer {
    scene: ViewerScene
    constructor(options?: {
      divID?: string
      width?: number
      height?: number
      background?: string
    })
    scaleToDimensions(width: number, height: number): void
  }

  class OccupancyGridClient {
    constructor(options?: { ros: Ros; rootObject: ViewerScene })
  }
}
