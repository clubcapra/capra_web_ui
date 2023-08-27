import { GlobalState } from '@/renderer/store/store';
import { createSlice, PayloadAction } from '@reduxjs/toolkit';
import { nanoid } from 'nanoid';

export interface GpioPinState {
  id: string;
  name: string;
  topicName: string;
  isOn: boolean;
  bpm: number;
}

export const initialState: GpioPinState[] = [
  {
    id: nanoid(),
    name: 'FRONT LED',
    topicName: '/DOP1',
    isOn: false,
    bpm: 0,
  },
  {
    id: nanoid(),
    name: 'BACK LED',
    topicName: '/DOP2',
    isOn: false,
    bpm: 0,
  },
  {
    id: nanoid(),
    name: 'ARMED LED',
    topicName: '/DOP3',
    isOn: false,
    bpm: 0,
  },
  {
    id: nanoid(),
    name: 'START ACTUATOR',
    topicName: '/DOP4',
    isOn: false,
    bpm: 136,
  },
];

export const gpioPinsSlice = createSlice({
  name: 'gpioPins',
  initialState,
  reducers: {
    togglePin: (state, { payload }: PayloadAction<string>) => {
      const element = state.find((element) => element.id === payload);
      if (element) {
        element.isOn = !element.isOn;
      }
    },
    addPin: (state, { payload }: PayloadAction<GpioPinState>) => {
      state.push(payload);
    },
    updatePin: (state, { payload }: PayloadAction<GpioPinState>) => {
      const element = state.find((element) => element.id === payload.id);
      if (element) {
        element.name = payload.name;
        element.topicName = payload.topicName;
      }
    },
    updateIsOn: (
      state,
      { payload }: PayloadAction<{ id: string; isOn: boolean }>
    ) => {
      const element = state.find((element) => element.id === payload.id);
      if (element) {
        element.isOn = payload.isOn;
      }
    },
    removePin: (state, { payload }: PayloadAction<GpioPinState>) => {
      state = state.filter((pin) => pin.id !== payload.id);
    },
    updateBPM: (state, { payload }: PayloadAction<{ id: string; bpm: number }>) => {
      const element = state.find((element) => element.id === payload.id);
      if (element) {
        element.bpm = payload.bpm;
      }
    },
  },
});

export const selectAllGpioPins = (state: GlobalState) => state.gpioPins;
