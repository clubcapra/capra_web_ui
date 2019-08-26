import RosClient from '@club_capra/roslib-ts-client'
import { store } from 'store/store'
import { rosSlice, fullRobotIpAddress } from 'store/modules/ros/reducer'

export const rosClient = new RosClient()
// rosClient.enableLogging()

const fullRobotIp = fullRobotIpAddress(store.getState())

rosClient.setListeners(
  () => {
    //onConnection
    store.dispatch(rosSlice.actions.setConnected(true))
  },
  () => {
    //onClose
    store.dispatch(rosSlice.actions.setConnected(false))
  },
  () => {
    // onError
    store.dispatch(
      rosSlice.actions.setError(`Failed to connect to: ${fullRobotIp}`)
    )
  }
)

rosClient.connect(fullRobotIp)
