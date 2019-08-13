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
      return {
        ...state,
        feeds: {
          ...state.feeds,
          [id]: {
            id,
            type: FeedType.video,
            camera: payload,
          },
        },
      }
    },
    removeCamera: (state, { payload }: PayloadAction<string>) => {
      const newState = { ...state }

      const newFeeds = { ...newState.feeds }
      delete newFeeds[payload]

      const newMap = { ...newState.feedMap }
      Object.values(newMap).forEach(m => {
        if (m.feedId === payload) {
          delete newMap[m.id]
        }
      })

      return {
        ...newState,
        feeds: { ...newFeeds },
        feedMap: newMap,
      }
    },
    changeCamera: (
      state,
      { payload: { camera, id } }: PayloadAction<{ camera: Camera; id: string }>
    ) => {
      const feed = { ...state.feeds[id] }

      if (feed.type !== FeedType.video) return { ...state }

      feed.camera = camera
      return { ...state, feeds: { ...state.feeds, [id]: feed } }
    },
    selectFedd: (
      state,
      { payload: { id, feedId } }: PayloadAction<{ id: string; feedId: string }>
    ) => {
      const newState = { ...state }
      newState.feedMap[id] = { id, feedId }
      return newState
    },
  },
})
