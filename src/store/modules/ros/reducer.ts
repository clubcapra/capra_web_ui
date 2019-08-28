import { RosState } from 'store/modules/ros/@types'
import { GlobalState } from 'store/rootReducer'
import { createSlice, PayloadAction } from 'redux-starter-kit'
import { toast } from 'react-toastify'
import { ICameraData } from 'store/modules/feed/@types'

export const initialState: RosState = {
  connected: false,
  IP: 'localhost',
  port: '9090',
  error: '',
  tryingToConnect: false,
  connectingToastId: '',
  errorToastId: '',
}

export const rosSlice = createSlice({
  initialState,
  reducers: {
    setIp: (state, { payload }: PayloadAction<string>) => {
      state.IP = payload
    },
    setPort: (state, { payload }: PayloadAction<string>) => {
      state.port = payload
    },
    setConnected: (state, { payload }: PayloadAction<boolean>) => {
      toast.dismiss(state.connectingToastId)
      state.tryingToConnect = false

      if (payload) {
        toast.info(`ROS: Connected to: ${formatIp(state)}`)
      } else if (state.connected) {
        toast.warn(`ROS: Lost connection to: ${formatIp(state)}`)
      }

      state.connected = payload
    },
    setError: (state, { payload }: PayloadAction<unknown>) => {
      state.errorToastId = toast.error(`ROS: ${payload}`)
      state.error = payload
    },
    tryToConnect: state => {
      toast.dismiss(state.errorToastId)
      state.connectingToastId = toast.warn(
        `ROS: Trying to connect to: ${formatIp(state)}`
      )
      state.tryingToConnect = true
    },
  },
})

const formatIp = (state: RosState): string =>
  `https://${state.IP}:${state.port}/`

export const fullIpAddress = (state: GlobalState): string => formatIp(state.ros)

export const selectVideoStreamUrl = (camera: ICameraData) => (
  state: GlobalState
): string =>
  `https://${state.ros.IP}:${state.ros.port}/stream` +
  `?topic=${camera.topic}&type=${camera.type}`
