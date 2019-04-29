import { Module, VuexModule, Mutation, getModule } from 'vuex-module-decorators'
import store from '..'
import { Vector3 } from '@/utils/math/types'
import { TopicWithData } from '@/utils/ros/types'

@Module({ dynamic: true, store, name: 'dashboard', namespaced: true })
class DashboardModule extends VuexModule {
  orientation: TopicWithData<Vector3> = {
    topic: {
      name: '/capra/imu',
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
      name: '/capra/imu',
      messageType: 'sensor_msgs/Imu',
    },
    data: 0,
  }

  speed: TopicWithData<number> = {
    topic: {
      name: '/capra/speed',
      messageType: 'vel',
    },
    data: 2,
  }

  @Mutation
  setOrientation(orientation: Vector3) {
    this.orientation.data = orientation
  }

  @Mutation
  setTemperature(temp: number) {
    this.temperature.data = temp
  }

  @Mutation
  setSpeed(speed: number) {
    this.speed.data = speed
  }
}

export default getModule(DashboardModule)
