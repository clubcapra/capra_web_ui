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
    name: 'Common',
    packageName: 'markhor_bringup',
    fileName: 'markhor_common.launch',
    isLaunched: false,
  },
  {
    name: 'Movement',
    packageName: 'markhor_bringup',
    fileName: 'markhor_movement.launch',
    isLaunched: false,
  },
  {
    name: 'Arm',
    packageName: 'ovis_bringup',
    fileName: 'ovis_arm.launch',
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

export const selectAllLaunchFiles = (state: GlobalState): LaunchFilesState[] =>
  state.launchFiles
