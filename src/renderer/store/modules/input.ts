import { GlobalState } from '@/renderer/store/store';
import { createSlice } from '@reduxjs/toolkit';

export interface InputState {
  reverse: boolean;
}

export const initialState: InputState = {
  reverse: false,
};

export const inputSlice = createSlice({
  name: 'input',
  initialState,
  reducers: {
    toggleReverse: (state) => {
      state.reverse = !state.reverse;
    },
  },
});

export const selectReverse = (state: GlobalState): boolean =>
  state.input.reverse;
