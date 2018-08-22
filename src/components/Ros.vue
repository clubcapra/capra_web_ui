<template lang="pug">
#ros
  .field.has-addons
    .control
      input.input.is-small(v-model="robotIP" v-on:keydown.enter="connect" :class="connected ? 'is-success' : 'is-danger'")
    .control
      button.button.is-small(v-on:click="connect" :class="connected ? 'is-success' : 'is-danger'") connect  
</template>

<script>
import RosClient from '@/lib/RosClient'
import { mapState, mapActions } from 'vuex'
import { rosActions } from '@/store/modules/ros'

let rosclient = null

export default {
  name: 'Ros',
  computed: {
    ...mapState({
      connected: state => state.ros.connected,
      error: state => state.ros.error
    }),
    robotIP: {
      get() {
        return this.$store.state.ros.robotIP
      },
      set(value) {
        this.handleIP(value)
      }
    }
  },
  mounted() {
    rosclient = new RosClient()
    rosclient.setListeners({
      connect: this.handleConnect,
      disconnect: this.handleDisconnect,
      error: this.handleError,
      updateOrientation: this.handleOrientation,
      updateTemperature: this.handleTemperature
    })
    rosclient.connect(this.robotIP)
  },
  methods: {
    ...mapActions('ros', {
      handleConnect: rosActions.CONNECT,
      handleDisconnect: rosActions.DISCONNECT,
      handleError: rosActions.ERROR,
      handleOrientation: rosActions.UPDATE_ORIENTATION,
      handleTemperature: rosActions.UPDATE_TEMPERATURE,
      handleIP: rosActions.UPDATE_IP
    }),
    connect: function() {
      rosclient.connect(this.robotIP)
    }
  }
}
</script>

<style lang="stylus" scoped>
</style>
