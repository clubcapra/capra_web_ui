<template>
  <div class="section">
    <p class="title">ROS</p>
    <div class="field">
      <label class="label">Robot IP</label>
      <div class="field has-addons">
        <div class="control">
          <base-input
            v-model="currentIP"
            :class="`is-small ${connectedClass}`"
            @keydown:enter="connect"
          />
        </div>
        <div class="control">
          <b-button
            :class="connectedClass"
            class="button is-small"
            @click="connect"
          >
            connect
          </b-button>
        </div>
      </div>
    </div>
  </div>
</template>

<script>
import { mapActions, mapState } from 'vuex'

export default {
  name: 'RosConfig',
  inject: ['rosClient'],
  computed: {
    ...mapState('ros', {
      connected: state => state.connected,
      robotIP: state => state.robotIP
    }),
    currentIP: {
      get() {
        return this.robotIP
      },
      set(robotIP) {
        this.setRobotIP(robotIP)
      }
    },
    connectedClass() {
      return this.connected ? 'is-success' : 'is-danger'
    }
  },
  methods: {
    ...mapActions('ros', ['setRobotIP']),
    connect() {
      this.rosClient.connect(this.robotIP)
    }
  }
}
</script>

<style lang="scss" scoped></style>
