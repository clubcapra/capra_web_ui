import { createSlice } from 'redux-starter-kit'
import { GamepadState } from 'store/modules/gamepad/@types'

export const initialState: GamepadState = {
  isArmControlled: true,
}

export const gamepadSlice = createSlice({
  initialState,
  reducers: {
    toggleIsArmControlled: state => {
      state.isArmControlled = !state.isArmControlled
    },
  },
})
