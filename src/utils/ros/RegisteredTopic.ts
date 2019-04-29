import { Topic } from 'roslib'
import { TopicOptions } from './types'

export default class RegisteredTopic {
  handlers: Array<Function> = []
  listener: Topic | undefined | null
  options: TopicOptions

  constructor(options: TopicOptions, handler: Function) {
    this.handlers = [handler]
    this.listener = undefined
    this.options = options
  }
}
