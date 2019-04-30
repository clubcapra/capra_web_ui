import { Ros, Topic, Message } from 'roslib'
import { TopicOptions } from './types'
import RegisteredTopic from './RegisteredTopic'

class TopicManager {
  private ros: Ros
  private registeredTopics: Map<string, RegisteredTopic> = new Map()
  private topics: Map<string, Topic> = new Map()

  constructor(ros: Ros) {
    this.ros = ros
  }

  listen(options: TopicOptions) {
    const signature = this.getSignature(options)
    const registeredTopic = this.registeredTopics.get(signature)

    if (registeredTopic) {
      const topic = this.getTopic(options)

      registeredTopic.topic = topic

      topic.subscribe(message => {
        registeredTopic.handlers.forEach(h => h(message))
      })
    }
  }

  subscribe(options: TopicOptions, handler: Function) {
    const signature = this.getSignature(options)
    const topic = this.registeredTopics.get(signature)

    if (topic) {
      topic.handlers.push(handler)
      return
    }

    this.registeredTopics.set(signature, new RegisteredTopic(options, handler))

    this.listen(options)
  }

  publish({ name, messageType }: TopicOptions, payload: any) {
    // console.log('RosClient publish() -> name:', name, ' message:', payload)
    this.getTopic({ name, messageType }).publish(new Message(payload))
  }

  unsubscribeAllTopics() {
    this.registeredTopics.forEach(topic => {
      if (topic.topic) {
        topic.topic.unsubscribe()
        topic.topic = null
      }
    })
  }

  reconnectAllDisconnectedHandler() {
    this.registeredTopics.forEach(topic => {
      if (topic.topic === null && topic.handlers.length > 0) {
        this.listen(topic.options)
        topic.topic = null
      }
    })
  }

  private getSignature({ name, messageType }: TopicOptions) {
    return `${name}/${messageType}`
  }

  private getTopic({ name, messageType }: TopicOptions) {
    const signature = this.getSignature({ name, messageType })

    if (this.topics.has(signature)) {
      return this.topics.get(signature) as Topic
    }
    const topic = new Topic({
      ros: this.ros,
      name,
      messageType,
    })

    this.topics.set(signature, topic)

    return topic
  }
}

export default TopicManager
