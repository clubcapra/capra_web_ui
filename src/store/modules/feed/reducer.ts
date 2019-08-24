import { Camera } from './@types'
import { PayloadAction, createSlice } from 'redux-starter-kit'
import { FeedType } from 'store/modules/feed/@types'
import { initialState } from 'store/modules/feed/initialState'
import shortid from 'shortid'

export const feedSlice = createSlice({
  initialState,
  reducers: {
    addCamera: (state, { payload }: PayloadAction<Camera>) => {
      const id = shortid()
      state.feeds[id] = {
        id,
        type: FeedType.video,
        camera: payload,
      }
    },
    removeFeed: (state, { payload }: PayloadAction<string>) => {
      delete state.feeds[payload]

      Object.values(state.feedMap).some(m => {
        if (m.feedId === payload) {
          delete state.feedMap[m.id]
          return true
        }
        return false
      })
    },
    changeCamera: (
      state,
      { payload: { camera, id } }: PayloadAction<{ camera: Camera; id: string }>
    ) => {
      const feed = state.feeds[id]

      if (feed.type !== FeedType.video) return
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
