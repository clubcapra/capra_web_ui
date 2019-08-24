export interface FeedState {
  feeds: FeedCollection
  feedMap: FeedMap
}

//TODO use actual Map
export interface FeedMap {
  [id: string]: {
    id: string
    feedId: string
  }
}

export enum FeedType {
  video,
  minimap2D,
  minimap3D,
  model,
  joystick,
}

export enum CameraType {
  img,
  png,
  vp8,
}

export interface FeedCollection {
  [feedId: string]: Feed
}

export type Feed =
  | CameraFeed
  | Minimap2DFeed
  | Minimap3DFeed
  | ModelFeed
  | JoystickFeed

interface IFeed {
  type: FeedType
  id: string
}

export interface CameraFeed extends IFeed {
  type: FeedType.video
  id: string
  camera: Camera
}

export interface Camera {
  name: string
  type: CameraType
  topic: string
}

export interface Minimap2DFeed extends IFeed {
  type: FeedType.minimap2D
}

export interface Minimap3DFeed extends IFeed {
  type: FeedType.minimap3D
}

export interface ModelFeed extends IFeed {
  type: FeedType.model
  id: string
}

export interface JoystickFeed extends IFeed {
  type: FeedType.joystick
  id: string
}
