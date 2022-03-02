import { GlobalState } from '@/renderer/store/store'
import { createSlice, PayloadAction } from '@reduxjs/toolkit'

export interface LaunchFilesState {
  name: string
  packageName: string
  fileName: string
  isLaunched: boolean
}

export const initialState: LaunchFilesState[] = [
  {
    name: 'Flippers',
    packageName: 'capra_tutorial',
    fileName: 'yes.launch',
    isLaunched: false,
  },
  {
    name: 'Tracks',
    packageName: 'markhor_bringup',
    fileName: 'markhor_base',
    isLaunched: false,
  },
]

export const launchFilesSlice = createSlice({
  name: 'launchFiles',
  initialState,
  reducers: {
    launchFile: (state, action: PayloadAction<string>) => {
      const element = state.find(
        (element) => element.fileName === action.payload
      )
      if (element) {
        element.isLaunched = true
      }
    },
    killFile: (state, action: PayloadAction<string>) => {
      const element = state.find(
        (element) => element.fileName === action.payload
      )
      if (element) {
        element.isLaunched = false
      }
    },
  },
})

export const selectAllLaunchFiles = (state: GlobalState) => state.launchFiles
