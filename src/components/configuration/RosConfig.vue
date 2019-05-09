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
            :is-danger="notConnected"
            :is-success="connected"
            @click="connect"
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
    // FIXME use this when my PR in vue-bulma-components is merged and released
    return this.connected ? 'is-success' : 'is-danger'
  }

  get connected() {
    return rosModule.connected
  }

  get notConnected() {
    return !this.connected
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
