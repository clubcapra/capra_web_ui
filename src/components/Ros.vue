<template>
  <div id="ros">
    <div>connected: {{ connected }}</div>
    <pre id="error">{{ error }}</pre>
  </div>
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
      updateTwist: this.handleTwist,
      updatePose: this.handlePose
    })
    rosclient.connect()
  },
  methods: {
    ...mapActions('ros', {
      handleConnect: rosActions.CONNECT,
      handleDisconnect: rosActions.DISCONNECT,
      handleError: rosActions.ERROR,
      handleTwist: rosActions.UPDATE_TWIST,
      handlePose: rosActions.UPDATE_POSE
    })
  }
}
</script>

<style lang="stylus" scoped>
#error
  color red
</style>
