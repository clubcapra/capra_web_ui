import RosClient, { TopicOptions } from './@types'
import RegisteredTopic from './RegisteredTopic'
import { getTopicSignature } from './getSignature'
import ROSLIB from 'roslib'
import type { Ros, Topic } from 'roslib'
import { store } from '@/renderer/store/store'

class TopicManager {
  private ros: Ros
  private registeredTopics = new Map<string, RegisteredTopic>()
  private topics = new Map<string, Topic>()
  private client: RosClient

  constructor(ros: Ros, client: RosClient) {
    this.ros = ros
    this.client = client
  }

  subscribe<T>(
    options: TopicOptions<T>,
    handler: (message: { data: T }) => void
  ) {
    const signature = getTopicSignature(options)
    const topic = this.registeredTopics.get(signature)

    if (topic) {
      topic.handlers.push(handler)
      return
    }

    this.registeredTopics.set(signature, new RegisteredTopic(options, handler))

    this.listen(options)
  }

  unsubscribe(options: TopicOptions) {
    this.getTopic(options).unsubscribe()
  }

  publish({ name, messageType }: TopicOptions, payload: unknown) {
    const topicName = `/${name}`.replace(/\/\/+/g, '/')
    const topic = this.getTopic({
      name: topicName,
      messageType,
    })
    if (this.client.isLogEnabled) {
      // eslint-disable-next-line no-console
      console.log(topic.name, topicName, store.getState().ros.namespace, name)
    }
    topic.publish(new ROSLIB.Message(payload))
  }

  unsubscribeAllTopics() {
    this.registeredTopics.forEach((registeredTopic) => {
      if (registeredTopic.topic) {
        registeredTopic.topic.unsubscribe()
        registeredTopic.topic = null
      }
    })
  }

  reconnectAllDisconnectedHandler() {
    this.registeredTopics.forEach((topic) => {
      if (topic.topic === null && topic.handlers.length > 0) {
        this.listen(topic.options)
        topic.topic = null
      }
    })
  }

  private listen(options: TopicOptions) {
    const signature = getTopicSignature(options)
    const registeredTopic = this.registeredTopics.get(signature)

    if (registeredTopic) {
      const topic = this.getTopic(options)

      registeredTopic.topic = topic

      topic.subscribe((message) => {
        registeredTopic.handlers.forEach((h) => h(message))
      })
    }
  }

  private getTopic({ name, messageType }: TopicOptions) {
    const signature = getTopicSignature({ name, messageType })

    if (this.topics.has(signature)) {
      return this.topics.get(signature) as Topic
    }

    const topic = new ROSLIB.Topic({
      ros: this.ros,
      name,
      messageType,
    })

    this.topics.set(signature, topic)

    return topic
  }
}

export default TopicManager
