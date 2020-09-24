import RosClient from './RosClient'

export default RosClient

// eslint-disable-next-line @typescript-eslint/no-unused-vars
export interface TopicOptions<T = unknown> {
  name: string
  messageType: string
  compression?: string
  throttle_rate?: number
  queue_size?: number
  latch?: boolean
  queue_length?: number
}

export interface TopicWithData<T> {
  topic: TopicOptions
  data: T
}

export interface ServiceOptions {
  name: string
  serviceType?: string
}

export type Handler<T> = (message: { data: T }) => void
