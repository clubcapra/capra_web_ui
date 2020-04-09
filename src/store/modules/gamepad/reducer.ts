import { createSlice, PayloadAction } from '@reduxjs/toolkit'
import { GamepadState } from 'store/modules/gamepad/@types'
import { GlobalState } from 'store/rootReducer'

export const initialState: GamepadState = {
  isArmControlled: false,
}

export const gamepadSlice = createSlice({
  name: 'gamepad',
  initialState,
  reducers: {
    setIsArmControlled: (state, { payload }: PayloadAction<boolean>) => {
      state.isArmControlled = payload
    },
    toggleIsArmControlled: (state) => {
      state.isArmControlled = !state.isArmControlled
    },
  },
})

export const isArmControlledSelector = (state: GlobalState): boolean =>
  state.gamepad.isArmControlled
