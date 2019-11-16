import RosClient from './roslib-ts-client/RosClient'
import { store } from 'store/store'
import { rosSlice, fullIpAddress } from 'store/modules/ros/reducer'

const rosState = store.getState().ros
export const rosClient = new RosClient(rosState.IP, rosState.port)

rosClient.setListeners({
  onConnection: () => {
    store.dispatch(rosSlice.actions.setConnected(true))
  },
  onClose: () => {
    store.dispatch(rosSlice.actions.setConnected(false))
  },
  onError: () => {
    store.dispatch(
      rosSlice.actions.setError(
        `Failed to connect to: ${fullIpAddress(store.getState())}`
      )
    )
  },
})
