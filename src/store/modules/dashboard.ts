import { VuexModule, mutation, Module } from 'vuex-class-component'
import { Vector3 } from '@/utils/math/types'
import { TopicWithData } from '@/utils/ros/types'

@Module({ namespacedPath: 'dashboard/' })
export default class DashboardModule extends VuexModule {
  orientation: TopicWithData<Vector3> = {
    topic: {
      name: '/vectornav/IMU',
      messageType: 'sensor_msgs/Imu',
    },
    data: {
      x: 0,
      y: 0,
      z: 0,
    },
  }

  temperature: TopicWithData<number> = {
    topic: {
      name: '/vectornav/IMU',
      messageType: 'sensor_msgs/Imu',
    },
    data: 0,
  }

  @mutation
  setOrientation(orientation: Vector3) {
    this.orientation.data = orientation
  }

  @mutation
  setTemperature(temp: number) {
    this.temperature.data = temp
  }
}
