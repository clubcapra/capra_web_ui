import RosClient from '@club_capra/roslib-ts-client'
import { store } from 'store/store'
import { rosSlice, fullIpAddress } from 'store/modules/ros/reducer'

export const rosClient = new RosClient()

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

const rosState = store.getState().ros
rosClient.connect(rosState.IP, rosState.port)
