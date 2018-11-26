<template>
  <div />
</template>

<script>
import RosClient from '@/lib/RosClient'
import { mapActions } from 'vuex'
import {
  CONNECT,
  DISCONNECT,
  UPDATE_ORIENTATION,
  UPDATE_TEMPERATURE
} from '@/store/actions.type'

let rosclient = null

export default {
  name: 'Ros',
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
    ...mapActions('ROS', {
      onConnection: CONNECT,
      onClose: DISCONNECT,
      handleUpdateOrientation: UPDATE_ORIENTATION,
      handleUpdateTemperature: UPDATE_TEMPERATURE
    }),
    connect() {
      rosclient.disconnect()
      rosclient.connect(this.robotIP)
    }
  }
}
</script>
