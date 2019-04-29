export interface TopicOptions {
  name: string
  messageType: string
}

export interface TopicWithData<T> {
  topic: TopicOptions
  data: T
}
