<template>
  <b-section>
    <b-title>ROS</b-title>
    <hr />

    <input-with-label v-model="currentIP" label="Robot IP" />

    <input-with-label v-model="port" label="Port" />

    <b-button :class="connectedClass" @click="connect">Connect</b-button>

    <br />
    <br />

    <b-button class="is-danger" @click="clearCache">Clear Cache</b-button>
  </b-section>
</template>

<script lang="ts">
import { Vue, Component, Inject } from 'vue-property-decorator'
import RosClient from '@/utils/ros/RosClient'
import { InputWithLabel } from '@/components/ui'

import { rosModule } from '@/store'

@Component({ components: { InputWithLabel } })
export default class RosConfig extends Vue {
  get connectedClass() {
    if (rosModule.connecting) return 'is-warning is-loading'
    return this.connected ? 'is-success' : 'is-danger'
  }

  get connected() {
    return rosModule.connected
  }

  get currentIP() {
    return rosModule.robotIP
  }

  set currentIP(value: string) {
    rosModule.setRobotIP(value)
  }

  get port() {
    return rosModule.port
  }

  set port(value: string) {
    rosModule.setPort(value)
  }

  connect() {
    rosModule.connect()
  }

  clearCache() {
    localStorage.removeItem('vuex')
  }
}
</script>
