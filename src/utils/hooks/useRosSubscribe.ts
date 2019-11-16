import { rosClient } from 'utils/ros/rosClient'
import { useEffect } from 'react'
import { useSelector } from 'utils/hooks/typedUseSelector'
import { TopicOptions } from 'utils/ros/roslib-ts-client/@types'

/**
 * Subscribes to a specified topic
 * The callback will be called everytime a topic receives new data
 *
 * This will automatically unsubscribe when the component is unmounted
 */
export const useRosSubscribe = <R>(
  topic: TopicOptions<R>,
  callback: (message: { data: R }) => void
) => {
  const connected = useSelector(state => state.ros.connected)

  useEffect(() => {
    if (connected) {
      rosClient.subscribe(topic, callback)
      return rosClient.unsubscribe(topic)
    }
  }, [callback, connected, topic])
}
