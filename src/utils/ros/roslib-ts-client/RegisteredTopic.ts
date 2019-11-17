import { Topic } from 'roslib'
import { TopicOptions } from './@types'

export default class RegisteredTopic {
  handlers: Function[] = []
  topic: Topic | undefined | null
  options: TopicOptions

  constructor(options: TopicOptions, handler: Function) {
    this.handlers = [handler]
    this.topic = undefined
    this.options = options
  }
}
