<template lang="pug">
#ros
  .field.has-addons
    .control
      input.input.is-small(
        v-model="currentIP"
        v-on:keydown.enter="connect"
        :class="connectedClass"
      )
    .control
      button.button.is-small(
        v-on:click="connect"
        :class="connectedClass"
      ) connect
</template>

<script>
import RosClient from '@/lib/RosClient'
import { mapState, mapActions } from 'vuex'
import {
  CONNECT,
  DISCONNECT,
  UPDATE_ROBOTIP,
  UPDATE_ORIENTATION,
  UPDATE_TEMPERATURE
} from '@/store/actions.type'

let rosclient = null

export default {
  name: 'Ros',
  computed: {
    ...mapState({
      connected: state => state.ros.connected,
      robotIP: state => state.ros.robotIP
    }),
    currentIP: {
      get() {
        return this.robotIP
      },
      set(robotIP) {
        this.handleUpdateRobotIP(robotIP)
      }
    },
    connectedClass() {
      return this.connected ? 'is-success' : 'is-danger'
    }
  },
  mounted() {
    rosclient = new RosClient()
    rosclient.setListeners({
      onConnection: this.onConnection,
      onClose: this.onClose
    })
    rosclient.setSubscribers({
      updateOrientation: this.handleUpdateOrientation,
      updateTemperature: this.handleUpdateTemperature
    })
    this.connect()
  },
  methods: {
    ...mapActions('ros', {
      onConnection: CONNECT,
      onClose: DISCONNECT,
      handleUpdateOrientation: UPDATE_ORIENTATION,
      handleUpdateTemperature: UPDATE_TEMPERATURE,
      handleUpdateRobotIP: UPDATE_ROBOTIP
    }),
    connect() {
      this.disconnect()
      rosclient.connect(this.robotIP)
    },
    disconnect() {
      rosclient.disconnect()
    }
  }
}
</script>

<style lang="stylus" scoped>
</style>
