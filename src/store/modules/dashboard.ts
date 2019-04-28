import { Module, VuexModule, Mutation, Action } from 'vuex-module-decorators'
import store from '..'

type Topic = { name: string; messageType: string }
type Field = { topic: Topic; data: any }

@Module({ dynamic: true, store, name: 'dsahboard', namespaced: true })
export default class DashboardModule extends VuexModule {
  orientation: Field = {
    topic: {
      name: '/capra/imu',
      messageType: 'sensor_msgs/Imu',
    },
    data: <Vector3>{
      x: 0,
      y: 0,
      z: 0,
    },
  }

  temperature: Field = {
    topic: {
      name: '/capra/imu',
      messageType: 'sensor_msgs/Imu',
    },
    data: 0,
  }

  speed: Field = {
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
  setTemperature(temp: Number) {
    this.temperature.data = temp
  }

  @Mutation
  setSpeed(speed: Number) {
    this.speed.data = speed
  }
}
