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
    ...mapState('ros', {
      connected: state => state.connected,
      robotIP: state => state.robotIP
    }),
    currentIP: {
      get() {
        return this.robotIP
      },
      set(robotIP) {
        console.log(robotIP)
        this.setRobotIP(robotIP)
      }
    },
    connectedClass() {
      return this.connected ? 'is-success' : 'is-danger'
    }
  },
  methods: {
    ...mapActions('ros', {
      setRobotIP: 'setRobotIP'
    }),
    connect() {
      this.rosClient.connect(this.robotIP)
    }
  }
}
</script>

<style lang="scss" scoped></style>
