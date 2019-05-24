<template>
  <b-section>
    <b-title>ROS</b-title>
    <hr />

    <input-with-button
      v-model="currentIP"
      label="Robot IP"
      button-text="Connect"
      :class="connectedClass"
      @click="connect"
    />
  </b-section>
</template>

<script lang="ts">
import { Vue, Component, Inject } from 'vue-property-decorator'
import RosClient from '@/utils/ros/RosClient'
import InputWithButton from '@/components/ui/InputWithButton.vue'

import { rosModule } from '@/store'

@Component({ components: { InputWithButton } })
export default class RosConfig extends Vue {
  @Inject('rosClient') rosClient!: RosClient

  get connectedClass() {
    return rosModule.connected ? 'is-success' : 'is-danger'
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
