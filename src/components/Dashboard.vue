<template>
  <div id="dashboard">
    <h3>IMU</h3>
    <div id="data">
      <div>x: {{ orientation.x }}</div>
      <div>y: {{ orientation.y }}</div>
      <div>z: {{ orientation.z }}</div>
      <div>temp: {{ temp }}</div>
      <div>speed: <progress-bar :value="speed" /></div>
    </div>
  </div>
</template>

<script lang="ts">
import { Vue, Component, Prop, Inject } from 'vue-property-decorator'
import { getModule } from 'vuex-module-decorators'

import _mapValues from 'lodash/mapValues'

import DashboardModule from '@/store/modules/dashboard'
import RosClient from '@/utils/ros/RosClient'

import ProgressBar from '@/components/UI/ProgressBar.vue'
import GamepadManager from '@/utils/gamepad/GamepadManager'
import { Stick, GamepadBtn } from '../utils/gamepad/mappings/types'

@Component({
  components: { ProgressBar },
})
export default class Dashboard extends Vue {
  @Inject('rosClient') rosClient!: RosClient
  @Inject() gamepadManager!: GamepadManager

  private readonly GAMEPAD_UPDATE_DELAY = 20

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
      const { gamepad } = this.gamepadManager
      this.speed =
        gamepad.getStick(Stick.Left).vertical *
        gamepad.getButtonValue(GamepadBtn.RT)
    }, this.GAMEPAD_UPDATE_DELAY)
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
