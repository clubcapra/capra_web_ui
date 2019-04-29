<template>
  <div class="section">
    <p class="title">ROS</p>
    <div class="field">
      <label class="label"> Robot IP </label>
      <div class="field has-addons">
        <div class="control">
          <input
            v-model="currentIP"
            :class="`input is-small ${connectedClass}`"
            @keydown:enter="connect"
          />
        </div>
        <div class="control">
          <button :class="`button is-small ${connectedClass}`" @click="connect">
            connect
          </button>
        </div>
      </div>
    </div>
  </div>
</template>

<script lang="ts">
import { Vue, Component, Inject } from 'vue-property-decorator'
import RosClient from '@/utils/ros/RosClient'

import RosModule from '@/store/modules/ros'

@Component
export default class RosConfig extends Vue {
  @Inject('rosClient') rosClient!: RosClient

  get connectedClass() {
    return RosModule.connected ? 'is-success' : 'is-danger'
  }

  get currentIP() {
    return RosModule.robotIP
  }

  set currentIP(event: any) {
    RosModule.setRobotIP(event.target.value)
  }

  connect() {
    this.rosClient.connect(this.currentIP)
  }
}
</script>

<style lang="scss" scoped></style>
