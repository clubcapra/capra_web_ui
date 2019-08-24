import { PWAState } from './@types'
import { createSlice } from 'redux-starter-kit'

export const initialState: PWAState = {
  contentLoaded: false,
  newContentLoaded: false,
  offlineMode: false,
}

export const pwaSlice = createSlice({
  initialState,
  reducers: {
    onContentLoaded: state => {
      state.contentLoaded = true
    },
    onNewContentLoaded: state => {
      state.newContentLoaded = true
    },
    onOfflineModeDetected: state => {
      state.offlineMode = true
    },
  },
})
