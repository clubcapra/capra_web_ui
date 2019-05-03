<template>
  <b-button is-small is-danger class="e-stop-btn" @click="sendServiceStop"
    >Stop !</b-button
  >
</template>

<script lang="ts">
import { Vue, Component, Inject } from 'vue-property-decorator'
import RosClient from '../utils/ros/RosClient'

@Component
export default class EStop extends Vue {
  @Inject() rosClient!: RosClient

  async sendServiceStop() {
    console.log('Stop')
    this.rosClient.callService({ name: 'takin_estop', serviceType: '' }, '')
    const result = await prompt('Robot is stopped.\n Want to restart?')
    if (result !== null) console.log('restart')
    else console.log('no restart')
  }
}
</script>

<style lang="scss" scoped>
.e-stop-btn {
  height: 100%;
  width: 100%;
}
</style>
