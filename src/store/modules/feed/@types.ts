export interface FeedState {
  feeds: FeedCollection
  feedMap: FeedMap
}

export interface FeedMap {
  [id: string]: FeedMapValue
}

export interface FeedMapValue {
  id: string
  feedId: string
}

export enum FeedTypeEnum {
  camera,
  minimap2D,
  minimap3D,
  model,
  joystick,
}

export enum CameraType {
  MJPEG = 'mjpeg',
  PNG = 'png',
  VP8 = 'vp8',
  WEBCAM = 'webcam',
}

export interface FeedCollection {
  [feedId: string]: FeedType
}

export type FeedType =
  | ICameraFeed
  | IMinimap2DFeed
  | IMinimap3DFeed
  | IModelFeed
  | IJoystickFeed

interface IFeed {
  type: FeedTypeEnum
  id: string
}

export interface ICameraFeed extends IFeed {
  type: FeedTypeEnum.camera
  id: string
  camera: ICameraData
}

export interface ICameraData {
  name: string
  type: CameraType
  topic: string
}

export interface IMinimap2DFeed extends IFeed {
  type: FeedTypeEnum.minimap2D
}

export interface IMinimap3DFeed extends IFeed {
  type: FeedTypeEnum.minimap3D
}

export interface IModelFeed extends IFeed {
  type: FeedTypeEnum.model
  id: string
}

export interface IJoystickFeed extends IFeed {
  type: FeedTypeEnum.joystick
  id: string
}
