import { PWAState } from './@types'
import { createSlice } from '@reduxjs/toolkit'
import { toast } from 'react-toastify'

export const initialState: PWAState = {
  contentLoaded: false,
  newContentLoaded: false,
  offlineMode: false,
}

export const pwaSlice = createSlice({
  name: 'pwa',
  initialState,
  reducers: {
    onContentLoaded: state => {
      toast.info('Content is cached for offline use.')
      state.contentLoaded = true
    },
    onNewContentLoaded: state => {
      toast.info(
        'New content is available and will be used when all tabs for this page are closed.'
      )
      state.newContentLoaded = true
    },
    onOfflineModeDetected: state => {
      toast.warn(
        'No internet connection found. App is running in offline mode.'
      )
      state.offlineMode = true
    },
  },
})
