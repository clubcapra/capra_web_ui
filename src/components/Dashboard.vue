<template>
  <div id="dashboard">
    <h3>IMU</h3>
    <div id="data">
      <div>x: {{ orientation.x }}</div>
      <div>y: {{ orientation.y }}</div>
      <div>z: {{ orientation.z }}</div>
      <div>temp: {{ temp }}</div>
      <div>speed: {{ speed }} m/s</div>
    </div>
  </div>
</template>

<script lang="ts">
import { Vue, Component, Prop, Inject } from 'vue-property-decorator'
import { getModule } from 'vuex-module-decorators'

import _mapValues from 'lodash/mapValues'

import DashboardModule from '@/store/modules/dashboard'
import RosClient from '@/utils/ros/RosClient'

@Component
export default class Dashboard extends Vue {
  @Inject('rosClient') rosClient!: RosClient

  speed = 0

  get orientation() {
    const orientation = DashboardModule.orientation.data
    const mapDirection = (dir: number) => dir.toFixed(4).padStart(6)
    return _mapValues(orientation, mapDirection)
  }

  get temp() {
    return DashboardModule.temperature.data
  }

  mounted() {
    this.rosClient.subscribe(
      DashboardModule.orientation.topic,
      DashboardModule.setOrientation
    )

    this.rosClient.subscribe(
      DashboardModule.temperature.topic,
      DashboardModule.setTemperature
    )

    setInterval(() => {
      this.speed = Math.floor(Math.random() * 10 + 1)
    }, 500)
  }
}
</script>

<style lang="scss" scoped>
#dashboard {
  #data {
    font-family: monospace;
    line-height: 1em;
  }
}
</style>
