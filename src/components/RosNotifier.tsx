import { FC, useEffect, useState } from 'react'
//@ts-ignore
import { useToasts } from 'react-toast-notifications'
import { useSelector } from 'utils/hooks/typedUseSelector'
import { fullRobotIpAddress } from 'store/modules/ros/reducer'

export const RosNotifier: FC = () => {
  const { addToast } = useToasts()

  const fullRobotIp = useSelector(fullRobotIpAddress)

  const error = useSelector(state => state.ros.error)
  const connected = useSelector(state => state.ros.connected)
  const [initialConnect, setInitialConnect] = useState(false)

  useEffect(() => {
    if (error !== '')
      addToast(`ROS: ${error}`, {
        appearance: 'error',
        autoDismiss: true,
      })
  }, [addToast, error])

  useEffect(() => {
    if (connected) {
      setInitialConnect(true)
      addToast(`ROS: Connected to: ${fullRobotIp}`, {
        appearance: 'success',
        autoDismiss: true,
      })
    } else if (initialConnect) {
      addToast(`ROS: Lost connection to: ${fullRobotIp}`, {
        appearance: 'error',
        autoDismiss: true,
      })
    }
  }, [addToast, connected, fullRobotIp, initialConnect])

  return null
}
