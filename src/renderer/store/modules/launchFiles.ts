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
    name: 'Parkour parkour',
    packageName: 'markhor_bringup',
    fileName: 'markhor_parkour.launch',
    isLaunched: false,
  },
  {
    name: "Observation (don't forget to launch the audio locally)",
    packageName: 'markhor_bringup',
    fileName: 'markhor_observation.launch',
    isLaunched: false,
  },
  {
    name: 'Base',
    packageName: 'markhor_bringup',
    fileName: 'markhor_base.launch',
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
