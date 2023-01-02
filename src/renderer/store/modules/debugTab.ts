import { GlobalState } from '@/renderer/store/store';
import { createSlice } from '@reduxjs/toolkit';

export interface DebugTab {
  visible: boolean;
}

export const initialState: DebugTab = {
  visible: false,
};

export const debugTabSlice = createSlice({
  name: 'debugTab',
  initialState,
  reducers: {
    toggleVisible: (state) => {
      state.visible = !state.visible;
    },
  },
});

export const selectDebugTabVisible = (state: GlobalState): boolean =>
  state.debugTab.visible;
