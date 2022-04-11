import { GlobalState } from '@/renderer/store/store'
import { createSlice } from '@reduxjs/toolkit'

export interface FlippersViewToggle {
  visible: boolean
}

export const initialState: FlippersViewToggle = {
  visible: false,
}

export const flippersViewToggleSlice = createSlice({
  name: 'flippersViewToggle',
  initialState,
  reducers: {
    toggleVisible: (state) => {
      state.visible = !state.visible
    },
    setVisible: (state) => {
      state.visible = true
    },
    setNotVisible: (state) => {
      state.visible = false
    },
  },
})

export const selectFlippersViewToggleVisible = (state: GlobalState): boolean =>
  state.flippersViewToggle.visible
