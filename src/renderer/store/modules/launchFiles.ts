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
    name: 'Base',
    packageName: 'markhor_bringup',
    fileName: 'markhor_base.launch',
    isLaunched: false,
  },
  {
    name: 'Flippers',
    packageName: 'markhor_flippers',
    fileName: 'flippers.launch',
    isLaunched: false,
  },
  {
    name: 'Tracks',
    packageName: 'markhor_tracks',
    fileName: 'tracks.launch',
    isLaunched: false,
  },
  {
    name: 'EStop',
    packageName: 'capra_estop',
    fileName: 'estop.launch',
    isLaunched: false,
  },
  {
    name: 'Teleop',
    packageName: 'markhor_bringup',
    fileName: 'teleop_twist_joy.launch',
    isLaunched: false,
  },
  {
    name: 'Astra Camera',
    packageName: 'astra_camera',
    fileName: 'multi_embedded.launch',
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
