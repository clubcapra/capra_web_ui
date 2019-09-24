import { createSlice } from 'redux-starter-kit'
import { GamepadState } from 'store/modules/gamepad/@types'
import { toast } from 'react-toastify'

export const initialState: GamepadState = {
  isArmControlled: false,
}

export const gamepadSlice = createSlice({
  initialState,
  reducers: {
    toggleIsArmControlled: state => {
      state.isArmControlled = !state.isArmControlled
      if (state.isArmControlled) {
        toast.info('Robot now in arm controlled mode')
      } else {
        toast.info('Robot now in movement mode')
      }
    },
  },
})
