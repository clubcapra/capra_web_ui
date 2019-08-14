import { createSlice } from 'redux-starter-kit'
import { GamepadState } from 'store/modules/gamepad/@types';

const initialState: GamepadState = {
  isArmControlled: true

}

export const gempadSlice = createSlice({
  initialState,
  reducers: {
    toggleIsArmControlled: state => ({
      ...state,
      isArmControlled: !state.isArmControlled,
    }),
  },
})
