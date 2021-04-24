import { initialState } from '@/renderer/store/modules/ros/initialState'
import { GlobalState } from '@/renderer/store/rootReducer'
import { createSlice, PayloadAction } from '@reduxjs/toolkit'

export const rosSlice = createSlice({
  name: 'ros',
  initialState,
  reducers: {
    updateNamespace: (state, { payload }: PayloadAction<string>) => {
      state.namespace = payload
    },
  },
})

export const selectNamespace = (state: GlobalState) => state.ros.namespace
