<template lang="pug">
#ros
  div(:style='{ color: connected ? "green" : "red" }')
    | connected: {{ connected }}
</template>

<script>
import RosClient from '@/lib/RosClient'
import { mapState, mapActions } from 'vuex'
import { rosActions } from '@/store/modules/ros'

export default {
  name: 'Ros',
  computed: mapState({
    connected: state => state.ros.connected,
    error: state => state.ros.error
  }),
  mounted() {
    let rosclient = new RosClient()
    rosclient.setListeners({
      connect: this.handleConnect,
      disconnect: this.handleDisconnect,
      error: this.handleError,
      updateOrientation: this.handleOrientation,
      updateCamera: this.handleCamera
    })
    rosclient.connect({
      updateCamera: this.handleCamera
    })
  },
  methods: {
    ...mapActions('ros', {
      handleConnect: rosActions.CONNECT,
      handleDisconnect: rosActions.DISCONNECT,
      handleError: rosActions.ERROR,
      handleOrientation: rosActions.UPDATE_ORIENTATION,
      handleCamera: rosActions.UPDATE_CAMERA
    })
  }
}
</script>

<style lang="stylus" scoped>
#error
  color red
</style>
