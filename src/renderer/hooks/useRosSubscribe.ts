import { rosClient } from '@/renderer/utils/ros/rosClient'
import { useEffect } from 'react'
import { TopicOptions } from '@/renderer/utils/ros/roslib-ts-client/@types'
import { useActor } from '@xstate/react'
import { rosService } from '@/renderer/state/ros'

/**
 * Subscribes to a specified topic
 * The callback will be called everytime a topic receives new data
 *
 * This will automatically unsubscribe when the component is unmounted
 *
 * It is recommended to wrap the callback parameter in a useCallback hook
 * otherwise it might subscribe and unsubscribe more than necessary.
 *  ```js
 *  useRosSubscribe(topic, useCallback((message) => {
 *    console.log(message)
 *  }, []))
 *  ```
 */
export function useRosSubscribe<T>(
  topic: TopicOptions<T>,
  callback: (message: { data: T }) => void
): void {
  const [state] = useActor(rosService)
  useEffect(() => {
    if (state.matches('connected')) {
      rosClient.subscribe(topic, callback)
      return () => rosClient.unsubscribe(topic)
    }
  }, [topic, state, callback])
}
