import { GlobalState } from '@/renderer/store/store';
import { createSlice, PayloadAction } from '@reduxjs/toolkit';

export interface LaunchFilesState {
  name: string;
  packageName: string;
  fileName: string;
  isLaunched: boolean;
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
  {
    name: 'Navigation',
    packageName: 'markhor_navigation',
    fileName: 'markhor_nav.launch',
    isLaunched: false,
  },
  {
    name: 'Slam Real',
    packageName: 'markhor_slam',
    fileName: 'markhor_slam_real.launch',
    isLaunched: false,
  },
  {
    name: 'Slam Real RGB',
    packageName: 'markhor_slam',
    fileName: 'markhor_slam_real.launch camera:=true',
    isLaunched: false,
  },
  {
    name: 'Slam Simulation',
    packageName: 'markhor_slam',
    fileName: 'markhor_slam.launch',
    isLaunched: false,
  },
  {
    name: 'Slam 2D',
    packageName: 'markhor_slam',
    fileName: 'markhor_slam_2d.launch',
    isLaunched: false,
  },
  {
    name: 'Radiation Sensor Mapping',
    packageName: 'capra_sensor_mapping',
    fileName: 'sensor_mapping.launch',
    isLaunched: false,
  },
];

export const launchFilesSlice = createSlice({
  name: 'launchFiles',
  initialState,
  reducers: {
    launchFile: (state, action: PayloadAction<string>) => {
      const element = state.find(
        (element) => element.fileName === action.payload
      );
      if (element) {
        element.isLaunched = true;
      }
    },
    killFile: (state, action: PayloadAction<string>) => {
      const element = state.find(
        (element) => element.fileName === action.payload
      );
      if (element) {
        element.isLaunched = false;
      }
    },
  },
});

export const selectAllLaunchFiles = (state: GlobalState): LaunchFilesState[] =>
  state.launchFiles;
