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
    name: 'LED 1',
    topicName: '',
    isOn: false,
  },
  {
    id: nanoid(),
    name: 'LED 2',
    topicName: '',
    isOn: false,
  },
  {
    id: nanoid(),
    name: 'LED 3',
    topicName: '',
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
    removePin: (state, { payload }: PayloadAction<GpioPinsState>) => {
      state = state.filter((pin) => pin.id !== payload.id);
    },
  },
});

export const selectAllGpioPins = (state: GlobalState) => state.gpioPins;
