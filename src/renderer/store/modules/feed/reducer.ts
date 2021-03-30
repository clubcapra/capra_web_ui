import { ICameraData, ICameraFeed } from './@types'
import { PayloadAction, createSlice } from '@reduxjs/toolkit'
import { FeedTypeEnum } from '@/renderer/store/modules/feed/@types'
import { initialState } from '@/renderer/store/modules/feed/initialState'
import shortid from 'shortid'
import { GlobalState } from '@/renderer/store/rootReducer'

export const feedSlice = createSlice({
  name: 'feed',
  initialState,
  reducers: {
    addCamera: (state, { payload }: PayloadAction<ICameraData>) => {
      const id = shortid()
      state.feeds[id] = {
        id,
        type: FeedTypeEnum.Camera,
        camera: payload,
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
    updateFeedMap: (
      state,
      { payload: { id, feedId } }: PayloadAction<{ id: string; feedId: string }>
    ) => {
      state.feedMap[id] = { id, feedId }
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
