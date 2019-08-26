import { RosState } from 'store/modules/ros/@types'
import { GlobalState } from 'store/rootReducer'
import { createSlice, PayloadAction } from 'redux-starter-kit'
import { toast } from 'react-toastify'

export const initialState: RosState = {
  connected: false,
  robotIp: 'localhost',
  robotPort: '9090',
  error: '',
}

export const rosSlice = createSlice({
  initialState,
  reducers: {
    setRobotIp: (state, { payload }: PayloadAction<string>) => {
      state.robotIp = payload
    },
    setRobotPort: (state, { payload }: PayloadAction<string>) => {
      state.robotPort = payload
    },
    setConnected: (state, { payload }: PayloadAction<boolean>) => {
      if (payload) {
        toast.info(`ROS: Connected to: ${formatIp(state)}`)
      } else if (state.connected) {
        toast.warn(`ROS: Lost connection to: ${formatIp(state)}`)
      }

      state.connected = payload
    },
    setError: (state, { payload }: PayloadAction<unknown>) => {
      toast.error(`ROS: ${payload}`)
      state.error = payload
    },
  },
})

const formatIp = (state: RosState): string =>
  `http://${state.robotIp}:${state.robotPort}/`

export const fullRobotIpAddress = (state: GlobalState): string =>
  formatIp(state.ros)
