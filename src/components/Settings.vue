<template>
  <div class="field has-addons">
    <div class="control">
      <input
        v-model="currentIP"
        :class="connectedClass"
        class="input is-small"
        @keydown.enter="connect">
    </div>
    <div class="control">
      <button
        :class="connectedClass"
        class="button is-small"
        @click="connect"
      >
        connect
      </button>
    </div>
  </div>
</template>

<script>
import RosClient from '@/lib/RosClient'
import { mapState, mapActions } from 'vuex'
import { UPDATE_ROBOTIP } from '@/store/actions.type'

let rosclient = null

export default {
  name: 'Settings',
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
  },
  methods: {
    ...mapActions('ros', {
      handleUpdateRobotIP: UPDATE_ROBOTIP
    }),
    connect() {
      rosclient.disconnect()
      rosclient.connect(this.robotIP)
    }
  }
}
</script>

<style lang="stylus" scoped>
</style>
