<template>
  <b-section>
    <b-title>ROS</b-title>
    <hr />

    <b-field>
      <b-label>Robot IP</b-label>
      <b-field has-addons>
        <b-control>
          <input
            v-model="currentIP"
            :class="`input is-small ${connectedClass}`"
            @keydown:enter="connect"
          />
        </b-control>
        <b-control>
          <b-button
            is-small
            @click="connect"
            :class="connectedClass"
            >Connect</b-button
          >
        </b-control>
      </b-field>
    </b-field>
  </b-section>
</template>

<script lang="ts">
import { Vue, Component, Inject } from 'vue-property-decorator'
import RosClient from '@/utils/ros/RosClient'

import { rosModule } from '@/store'

@Component
export default class RosConfig extends Vue {
  @Inject('rosClient') rosClient!: RosClient

  get connectedClass() {
    return this.connected ? 'is-success' : 'is-danger'
  }

  get currentIP() {
    return rosModule.robotIP
  }

  set currentIP(value: string) {
    rosModule.setRobotIP(value)
  }

  connect() {
    this.rosClient.connect(this.currentIP)
  }
}
</script>

<style lang="scss" scoped></style>
