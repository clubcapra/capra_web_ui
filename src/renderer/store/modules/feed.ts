import { GlobalState } from '@/renderer/store/store'
import { createSlice, PayloadAction } from '@reduxjs/toolkit'
import { TopicOptions } from '@/renderer/utils/ros/roslib-ts-client/@types'
import { nanoid } from 'nanoid'

export interface FeedState {
  feeds: { [feedId: string]: FeedType }
  feedMap: {
    [id: string]: {
      id: string
      feedId: string
    }
  }

  feed_front: string
  feed_back: string
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

export type FeedType = IEmptyFeed | ICameraFeed | IUrdfFeed | IGraphFeed

interface BaseFeed {
  type: FeedTypeEnum
  id: string
}

export interface IEmptyFeed extends BaseFeed {
  type: FeedTypeEnum.Empty
}

export interface ICameraFeed extends BaseFeed {
  type: FeedTypeEnum.Camera
  camera: ICameraData
}

export interface ICameraData {
  name: string
  type: CameraType
  topic: string
}

export interface IUrdfFeed extends BaseFeed {
  type: FeedTypeEnum.Urdf
}

export interface IGraphFeed extends BaseFeed {
  type: FeedTypeEnum.Graph
  graph: IGraphData
}

export interface IGraphData {
  topic: TopicOptions
  name: string
}

export const feed_id = {
  teleop: {
    main: 'teleop_main',
    bottom_left: 'teleop_bottom_left',
    top_left: 'teleop_top_left',
    top_right: 'teleop_top_right',
  },
  victim: {},
}

export const initialState: FeedState = {
  feedMap: {},
  feed_front: feed_id.teleop.main,
  feed_back: feed_id.teleop.bottom_left,
  feeds: {
    empty: {
      type: FeedTypeEnum.Empty,
      id: 'empty',
    },
    camera_3d_rgb: {
      type: FeedTypeEnum.Camera,
      id: 'camera_3d_rgb',
      camera: {
        name: 'camera_3d_rgb',
        type: CameraType.MJPEG,
        topic: '/camera_3d/rgb/image_raw',
      },
    },
    camera_3d_depth: {
      type: FeedTypeEnum.Camera,
      id: 'camera_3d_depth',
      camera: {
        name: 'camera_3d_depth',
        type: CameraType.MJPEG,
        topic: '/camera_3d/depth/image',
      },
    },
    webcam: {
      type: FeedTypeEnum.Camera,
      id: 'webcam',
      camera: {
        name: 'webcam',
        type: CameraType.WEBCAM,
        topic: '',
      },
    },
    urdf_viewer: {
      type: FeedTypeEnum.Urdf,
      id: 'urdf_viewer',
    },
    co2_graph: {
      type: FeedTypeEnum.Graph,
      id: 'co2_graph',
      graph: {
        name: 'co2_graph',
        topic: {
          name: '/capra/co2_ppm',
          messageType: 'std_msgs/String',
        },
      },
    },
  },
}

export const feedSlice = createSlice({
  name: 'feed',
  initialState,
  reducers: {
    addCamera: (state, { payload }: PayloadAction<ICameraData>) => {
      const id = nanoid()
      state.feeds[id] = {
        id,
        type: FeedTypeEnum.Camera,
        camera: payload,
      }
    },
    addGraph: (state, { payload }: PayloadAction<IGraphData>) => {
      const id = nanoid()
      state.feeds[id] = {
        id,
        type: FeedTypeEnum.Graph,
        graph: payload,
      }
    },
    removeFeed: (state, { payload }: PayloadAction<string>) => {
      delete state.feeds[payload]

      Object.values(state.feedMap).some((m) => {
        if (m.feedId === payload) {
          delete state.feedMap[m.id]
          return true
        }
        return false
      })
    },
    updateCamera: (
      state,
      {
        payload: { camera, id },
      }: PayloadAction<{ camera: ICameraData; id: string }>
    ) => {
      const feed = state.feeds[id]
      if (feed.type !== FeedTypeEnum.Camera) {
        return
      }
      feed.camera = camera
    },
    updateGraph: (
      state,
      {
        payload: { graph, id },
      }: PayloadAction<{ graph: IGraphData; id: string }>
    ) => {
      const feed = state.feeds[id]
      if (feed.type !== FeedTypeEnum.Graph) {
        return
      }
      feed.graph = graph
    },
    updateFeedMap: (
      state,
      { payload: { id, feedId } }: PayloadAction<{ id: string; feedId: string }>
    ) => {
      state.feedMap[id] = { id, feedId }
    },
    switchDirection: (state) => {
      const feed_id_old = state.feedMap[state.feed_front]
      state.feedMap[state.feed_front] = state.feedMap[state.feed_back]
      state.feedMap[state.feed_back] = feed_id_old
    },
  },
})

export const selectAllFeeds = (state: GlobalState) => state.feed.feeds

export const selectFeedFromFeedMap = (id: string) => (state: GlobalState) =>
  state.feed.feedMap[id]

export const selectAllCamera = (state: GlobalState) =>
  Object.values(state.feed.feeds)
    .filter((feed) => feed.type === FeedTypeEnum.Camera)
    .reverse() as ICameraFeed[]

export const selectAllGraph = (state: GlobalState) =>
  Object.values(state.feed.feeds)
    .filter((feed) => feed.type === FeedTypeEnum.Graph)
    .reverse() as IGraphFeed[]
