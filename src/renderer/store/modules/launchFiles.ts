import { GlobalState } from '@/renderer/store/store'
import { createSlice, PayloadAction } from '@reduxjs/toolkit'

export interface LaunchFilesState {
  name: string,
  fileName: string,
  isLaunched: boolean
}

export const initialState: LaunchFilesState[] = [{
    name: "Flippers",
    fileName: "markhor_flippers",
    isLaunched: false
},
{
    name: "Tracks",
    fileName: "markhor_base",
    isLaunched: false
}]

export const launchFilesSlice = createSlice({
    name: 'launchFiles',
    initialState,
    reducers: {
      launchFile: (state, action: PayloadAction<string>) => {
        const element = state.find(element => element.fileName === action.payload);
        if(element){
            element.isLaunched = !element.isLaunched;
        }
      },
    },
  });

  export const selectAllLaunchFiles = (state: GlobalState) =>
  state.launchFiles;

