import { RosState } from 'store/modules/ros/@types'
import { GlobalState } from 'store/rootReducer'
import { createSlice, PayloadAction } from 'redux-starter-kit'

export const initialState: RosState = {
  connected: false,
  robotIp: 'localhost',
  robotPort: '9090',
}

export const rosSlice = createSlice({
  initialState,
  reducers: {
    setRobotIp: (state, { payload }: PayloadAction<string>): RosState => ({
      ...state,
      robotIp: payload,
    }),
    setRobotPort: (state, { payload }: PayloadAction<string>): RosState => ({
      ...state,
      robotPort: payload,
    }),
    setConnected: (state, { payload }: PayloadAction<boolean>): RosState => ({
      ...state,
      connected: payload,
    }),
  },
})

export const fullRobotIpAddress = (state: GlobalState): string => {
  return `http://${state.ros.robotIp}:${state.ros.robotPort}/`
}
