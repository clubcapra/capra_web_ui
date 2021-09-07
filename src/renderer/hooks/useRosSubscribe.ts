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
 */
export const useRosSubscribe = <R>(
  topic: TopicOptions<R>,
  callback: (message: { data: R }) => void
): void => {
  const [state] = useActor(rosService)

  useEffect(() => {
    if (state.matches('connected')) {
      // eslint-disable-next-line @typescript-eslint/no-explicit-any
      rosClient.subscribe(topic, callback as any)
      return rosClient.unsubscribe(topic)
    }
  }, [callback, state, topic])
}
