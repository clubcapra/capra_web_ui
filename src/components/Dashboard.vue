<template>
  <div class="dashboard">
    <div class="data">
      <div>x: {{ orientation.x }}</div>
      <div>y: {{ orientation.y }}</div>
      <div>z: {{ orientation.z }}</div>
      <div>temp: {{ temp }}</div>
    </div>
    <div class="speed-progress">
      <progress-bar :value="forward" fill-parent vertical />
      <progress-bar :value="backward" fill-parent vertical reverse />
    </div>
    <div class="direction-progress">
      <progress-bar :value="left" reverse fill-parent />
      <progress-bar :value="right" fill-parent />
    </div>
  </div>
</template>

<script lang="ts">
import { Vue, Component, Prop, Inject } from 'vue-property-decorator'
import { getModule } from 'vuex-module-decorators'

import _ from 'lodash-es'

import DashboardModule from '@/store/modules/dashboard'
import RosClient from '@/utils/ros/RosClient'

import ProgressBar from '@/components/ui/ProgressBar.vue'
import GamepadManager from '@/utils/gamepad/GamepadManager'
import { Stick, GamepadBtn } from '../utils/gamepad/mappings/types'
import { mapGamepadToTwist } from '@/utils/math'

@Component({
  components: { ProgressBar },
})
export default class Dashboard extends Vue {
  @Inject('rosClient') rosClient!: RosClient
  @Inject() gamepadManager!: GamepadManager

  private readonly GAMEPAD_UPDATE_DELAY = 20

  speed = 0
  forward = 0
  backward = 0
  direction = 0
  right = 0
  left = 0

  get orientation() {
    const orientation = DashboardModule.orientation.data
    const mapDirection = (dir: number) => dir.toFixed(4).padStart(6)
    return _.mapValues(orientation, mapDirection)
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

      if (!gamepad) return

      const twist = mapGamepadToTwist(gamepad)

      this.forward = _.clamp(twist.linear.x, 0, 1)
      this.backward = Math.abs(_.clamp(twist.linear.x, -1, 0))

      this.left = Math.abs(_.clamp(twist.angular.z, -1, 0))
      this.right = _.clamp(twist.angular.z, 0, 1)
    }, this.GAMEPAD_UPDATE_DELAY)
  }
}
</script>

<style lang="scss" scoped>
.dashboard {
  display: grid;
  grid-template-rows: auto 15px;
  grid-template-columns: auto 15px;
  grid-template-areas:
    'a b'
    'c b';

  .data {
    grid-area: a;
    font-family: monospace;
    line-height: 1em;
  }

  .speed-progress {
    grid-area: b;
    display: grid;
    grid-template-rows: 1fr 1fr;
  }

  .direction-progress {
    grid-area: c;
    display: grid;
    grid-template-columns: 1fr 1fr;
  }
}
</style>
