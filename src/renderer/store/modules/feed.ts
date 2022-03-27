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
  NotSelected,
}

export enum CameraType {
  MJPEG = 'mjpeg',
  PNG = 'png',
  VP8 = 'vp8',
  WEBCAM = 'webcam',
}

export enum GraphType {
  GRAPH = 'graph',
  TEXT = 'text',
}

export type FeedType =
  | IEmptyFeed
  | ICameraFeed
  | IUrdfFeed
  | IGraphFeed
  | INotSelected

interface BaseFeed {
  type: FeedTypeEnum
  id: string
}

export interface IEmptyFeed extends BaseFeed {
  type: FeedTypeEnum.Empty
}

export interface INotSelected extends BaseFeed {
  type: FeedTypeEnum.NotSelected
}

export interface ICameraFeed extends BaseFeed {
  type: FeedTypeEnum.Camera
  camera: ICameraData
}

export interface ICameraData {
  name: string
  type: CameraType
  topic: string
  flipped: boolean
  rotated: boolean
}

export interface IUrdfFeed extends BaseFeed {
  type: FeedTypeEnum.Urdf
}

export interface IGraphFeed extends BaseFeed {
  type: FeedTypeEnum.Graph
  graph: IGraphData
}

export interface IGraphData {
  topic: TopicOptions<string>
  name: string
  type: GraphType
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
    not_selected: {
      type: FeedTypeEnum.NotSelected,
      id: 'not_selected',
    },
    video0: {
      type: FeedTypeEnum.Camera,
      id: 'video0',
      camera: {
        name: 'video0',
        type: CameraType.MJPEG,
        topic: '/markhor/video0/usb_cam/image_raw',
        flipped: true,
        rotated: false,
      },
    },
    video1: {
      type: FeedTypeEnum.Camera,
      id: 'video1',
      camera: {
        name: 'video1',
        type: CameraType.MJPEG,
        topic: '/markhor/video1/usb_cam/image_raw',
        flipped: true,
        rotated: false,
      },
    },
    video2: {
      type: FeedTypeEnum.Camera,
      id: 'video2',
      camera: {
        name: 'video2',
        type: CameraType.MJPEG,
        topic: '/markhor/video2/usb_cam/image_raw',
        flipped: true,
        rotated: true,
      },
    },
    video3: {
      type: FeedTypeEnum.Camera,
      id: 'video3',
      camera: {
        name: 'video3',
        type: CameraType.MJPEG,
        topic: '/markhor/video3/usb_cam/image_raw',
        flipped: true,
        rotated: false,
      },
    },
    front_cam: {
      type: FeedTypeEnum.Camera,
      id: 'front_cam',
      camera: {
        name: 'front_cam',
        type: CameraType.MJPEG,
        topic: '/markhor/front/usb_cam/image_raw',
        flipped: true,
        rotated: false,
      },
    },
    back_cam: {
      type: FeedTypeEnum.Camera,
      id: 'back_cam',
      camera: {
        name: 'back_cam',
        type: CameraType.MJPEG,
        topic: '/markhor/back/usb_cam/image_raw',
        flipped: true,
        rotated: false,
      },
    },
    arm_cam: {
      type: FeedTypeEnum.Camera,
      id: 'arm_cam',
      camera: {
        name: 'arm_cam',
        type: CameraType.MJPEG,
        topic: '/markhor/arm/usb_cam/image_raw',
        flipped: true,
        rotated: false,
      },
    },
    tpv_cam: {
      type: FeedTypeEnum.Camera,
      id: 'tpv_cam',
      camera: {
        name: 'tpv_cam',
        type: CameraType.MJPEG,
        topic: '/markhor/tpv/usb_cam/image_raw',
        flipped: true,
        rotated: false,
      },
    },
    webcam: {
      type: FeedTypeEnum.Camera,
      id: 'webcam',
      camera: {
        name: 'webcam',
        type: CameraType.WEBCAM,
        topic: '',
        flipped: false,
        rotated: false,
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
        type: GraphType.GRAPH,
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
