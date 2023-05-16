import { GlobalState } from '@/renderer/store/store';
import { createSlice, PayloadAction } from '@reduxjs/toolkit';
import { nanoid } from 'nanoid';

export interface GpioPinsState {
  id: string;
  name: string;
  topicName: string;
  isOn: boolean;
}

export const initialState: GpioPinsState[] = [
  {
    id: nanoid(),
    name: 'FRONT LED',
    topicName: '/DOP1',
    isOn: false,
  },
  {
    id: nanoid(),
    name: 'BACK LED',
    topicName: '/DOP2',
    isOn: false,
  },
  {
    id: nanoid(),
    name: 'ARM LED',
    topicName: '/DOP3',
    isOn: false,
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
    addPin: (state, { payload }: PayloadAction<GpioPinsState>) => {
      state.push(payload);
    },
    updatePin: (state, { payload }: PayloadAction<GpioPinsState>) => {
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
    removePin: (state, { payload }: PayloadAction<GpioPinsState>) => {
      state = state.filter((pin) => pin.id !== payload.id);
    },
  },
});

export const selectAllGpioPins = (state: GlobalState) => state.gpioPins;
