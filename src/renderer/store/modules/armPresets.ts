import { createSlice, PayloadAction } from '@reduxjs/toolkit';

export interface ArmPresetsState {
  presets: ArmPreset[];
  selectedPreset: ArmPreset;
}

export interface ArmPreset {
  name: string;
  positions: number[];
}

const initialPreset: ArmPreset = {
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
      state.presets = state.presets.filter((preset) => preset.name !== payload);
    },
    selectPreset: (state, { payload }: PayloadAction<string>) => {
      state.selectedPreset =
        state.presets.find((preset) => preset.name === payload) ??
        initialPreset;
    },
    updatePreset: (state, { payload }: PayloadAction<ArmPreset>) => {
      const index = state.presets.findIndex(
        (preset) => preset.name === payload.name
      );
      if (index !== -1) {
        state.presets[index] = payload;
      }
    },
  },
});
