import { RosState } from 'store/modules/ros/@types'
import { GlobalState } from 'store/rootReducer'
import { createSlice, PayloadAction } from '@reduxjs/toolkit'
import { toast } from 'react-toastify'
import { ICameraData } from 'store/modules/feed/@types'

export const initialState: RosState = {
  connected: false,
  IP: 'localhost',
  port: '9090',
  videoServerPort: '8080',
  error: '',
  tryingToConnect: false,
  connectingToastId: '',
  errorToastId: '',
  descriptionServerPort: '88',
  baseLinkName: 'markhor_link_base',
}

export const rosSlice = createSlice({
  name: 'ros',
  initialState,
  reducers: {
    setIp: (state, { payload }: PayloadAction<string>) => {
      state.IP = payload
    },
    setPort: (state, { payload }: PayloadAction<string>) => {
      state.port = payload
    },
    setVideoServerPort: (state, { payload }: PayloadAction<string>) => {
      state.videoServerPort = payload
    },
    setDescriptionServerPort: (state, { payload }: PayloadAction<string>) => {
      state.descriptionServerPort = payload
    },
    setBaseLinkName: (state, { payload }: PayloadAction<string>) => {
      state.baseLinkName = payload
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
      toast.dismiss(state.connectingToastId)
      state.connectingToastId = toast.warn(
        `ROS: Trying to connect to: ${formatIp(state)}`
      )
      state.tryingToConnect = true
    },
  },
})

const formatIp = (state: RosState): string => `${state.IP}:${state.port}/`

export const fullIpAddress = (state: GlobalState): string => formatIp(state.ros)

export const selectVideoUrl = (
  camera: ICameraData,
  param: 'stream' | 'snapshot' = 'stream'
) => (state: GlobalState): string =>
  `http://${state.ros.IP}:${state.ros.videoServerPort}/${param}` +
  `?topic=${camera.topic}&type=${camera.type}`
