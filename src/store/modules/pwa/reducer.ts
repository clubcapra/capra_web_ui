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
    onContentLoaded: (state): PWAState => ({
      ...state,
      contentLoaded: true,
    }),
    onNewContentLoaded: (state): PWAState => ({
      ...state,
      newContentLoaded: true,
    }),
    onOfflineModeDetected: (state): PWAState => ({
      ...state,
      offlineMode: true,
    }),
  },
})
