import { createSlice, PayloadAction } from '@reduxjs/toolkit';
import { nanoid } from 'nanoid';
import { GlobalState } from '../store';

export interface ArmPresetsState {
  presets: ArmPreset[];
  selectedPreset: ArmPreset;
}

export interface ArmPreset {
  id: string;
  name: string;
  positions: number[];
}

const initialPreset: ArmPreset = {
  id: nanoid(),
  name: 'Right Angle',
  positions: [180, 180, 180, 180, 180, 180],
};

export const initialState: ArmPresetsState = {
  presets: [initialPreset],
  selectedPreset: initialPreset,
};

export const armPresetsSlice = createSlice({
  name: 'armPresets',
  initialState,
  reducers: {
    addPreset: (state, { payload }: PayloadAction<ArmPreset>) => {
      state.presets.push(payload);
    },
    removePreset: (state, { payload }: PayloadAction<string>) => {
      state.presets = state.presets.filter((preset) => preset.id !== payload);
    },
    selectPreset: (state, { payload }: PayloadAction<string>) => {
      state.selectedPreset =
        state.presets.find((preset) => preset.id === payload) ?? initialPreset;
    },
    nextPreset: (state) => {
      const index = state.presets.findIndex(
        (preset) => preset.id === state.selectedPreset.id
      );
      if (index !== -1) {
        state.selectedPreset =
          state.presets[(index + 1) % state.presets.length] ?? initialPreset;
      }
    },
    updatePreset: (state, { payload }: PayloadAction<ArmPreset>) => {
      const index = state.presets.findIndex(
        (preset) => preset.id === payload.id
      );
      if (index !== -1) {
        state.presets[index] = payload;
      }
    },
  },
});

export const selectAllPresets = (state: GlobalState) =>
  state.armPresets.presets;

export const selectSelectedPreset = (state: GlobalState) =>
  state.armPresets.selectedPreset;
