import { rosClient } from 'utils/ros/rosClient'
import { TopicOptions } from '@club_capra/roslib-ts-client'
import { useEffect } from 'react'

export const useRosSubscribe = (topic: TopicOptions, callback: Function) => {
  useEffect(() => {
    rosClient.subscribe(topic, callback)
    return rosClient.unsubscribe(topic)
  }, [callback, topic])
}
