export interface FeedState {
  feeds: FeedCollection
  feedMap: FeedMap
  feed_front: string
  feed_back: string
}

export interface FeedMap {
  [id: string]: FeedMapValue
}

export interface FeedMapValue {
  id: string
  feedId: string
}

export enum FeedTypeEnum {
  Empty,
  Camera,
  Urdf,
  Graph,
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

export type FeedType = IEmptyFeed | ICameraFeed | IUrdfFeed | IGraphFeed

interface IFeed {
  type: FeedTypeEnum
  id: string
}

export interface IEmptyFeed extends IFeed {
  type: FeedTypeEnum.Empty
  id: string
}

export interface ICameraFeed extends IFeed {
  type: FeedTypeEnum.Camera
  id: string
  camera: ICameraData
}

export interface ICameraData {
  name: string
  type: CameraType
  topic: string
}

export interface IUrdfFeed extends IFeed {
  type: FeedTypeEnum.Urdf
  id: string
}

export interface IGraphFeed extends IFeed {
  type: FeedTypeEnum.Graph
  id: string
}
