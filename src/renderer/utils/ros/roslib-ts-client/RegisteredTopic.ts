import { Topic } from 'roslib'
import { TopicOptions } from './@types'

// eslint-disable-next-line @typescript-eslint/no-explicit-any
type Handler = (message: any) => void
export default class RegisteredTopic {
  handlers: Handler[] = []
  topic: Topic | undefined | null
  options: TopicOptions

  constructor(options: TopicOptions, handler: Handler) {
    this.handlers = [handler]
    this.topic = undefined
    this.options = options
  }
}
