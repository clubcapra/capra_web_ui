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
import _ from 'lodash'

import { Vue, Component, Prop, Inject } from 'vue-property-decorator'

import DashboardModule from '@/store/modules/dashboard'
import { getModule } from 'vuex-module-decorators'
import RosClient from '@/RosClient'

const dashboardModule = getModule(DashboardModule)

@Component
export default class Dashboard extends Vue {
  @Inject('rosClient') rosClient!: RosClient

  speed = 0

  get orientation() {
    const orientation = dashboardModule.orientation.data
    const mapDirection = (dir: number) => dir.toFixed(4).padStart(6)
    return _.mapValues(orientation, mapDirection)
  }

  get temp() {
    return dashboardModule.temperature.data
  }

  mounted() {
    this.rosClient.subscribe(
      dashboardModule.orientation.topic,
      dashboardModule.setOrientation
    )

    this.rosClient.subscribe(
      dashboardModule.temperature.topic,
      dashboardModule.setTemperature
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
