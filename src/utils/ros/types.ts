export interface TopicOptions {
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
  serviceType: string
}
