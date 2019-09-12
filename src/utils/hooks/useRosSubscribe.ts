import { rosClient } from 'utils/ros/rosClient'
import { TopicOptions } from '@club_capra/roslib-ts-client'
import { useEffect } from 'react'

/**
 * Subscribes to a specified topic
 * The callback will be called everytime a topic receives new data
 *
 * This will automatically unsubscribe when the component is unmounted
 */
export const useRosSubscribe = (topic: TopicOptions, callback: Function) => {
  useEffect(() => {
    rosClient.subscribe(topic, callback)
    return rosClient.unsubscribe(topic)
  }, [callback, topic])
}
