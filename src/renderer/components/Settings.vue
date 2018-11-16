<template>
  <div class="field has-addons">
    <div class="control">
      <input
        v-model="currentIP"
        :class="connectedClass"
        class="input is-small"
        @keydown.enter="connect"
      />
    </div>
    <div class="control">
      <button :class="connectedClass" class="button is-small" @click="connect">
        connect
      </button>
    </div>
  </div>
</template>

<script>
import { mapState, mapActions } from 'vuex'

export default {
  name: 'Settings',
  inject: ['rosClient'],
  computed: {
    ...mapState('ROS', {
      connected: state => state.connected,
      robotIP: state => state.robotIP
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
  methods: {
    ...mapActions('ROS', {
      handleUpdateRobotIP: 'setRobotIP'
    }),
    connect() {
      this.rosClient.disconnect()
      this.rosClient.connect(this.robotIP)
    }
  }
}
</script>
