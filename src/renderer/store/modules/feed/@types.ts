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
  urdf,
  empty,
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

export type FeedType = ICameraFeed | IUrdfFeed | IEmptyFeed

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

export interface IUrdfFeed extends IFeed {
  type: FeedTypeEnum.urdf
  id: string
}

export interface IEmptyFeed extends IFeed {
  type: FeedTypeEnum.empty
  id: string
}
