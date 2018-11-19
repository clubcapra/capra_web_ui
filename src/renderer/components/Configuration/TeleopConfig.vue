<template>
  <div class="section teleop-settings">
    <p class="title">Teleop</p>
    <div class="camera-container">
      <div class="card">
        <div class="card-header"><p class="card-header-title">Camera 1</p></div>
        <div class="card-content">
          <div class="field">
            <label>Camera 1 topic</label>
            <input v-model="camera1Topic" class="input is-small" />
          </div>
          <div class="field">
            <label>Camera 1 type</label>
            <input v-model="camera1Type" class="input is-small" />
          </div>
        </div>
      </div>
      <div class="card">
        <div class="card-header"><p class="card-header-title">Camera 2</p></div>
        <div class="card-content">
          <div class="field">
            <label>Camera 2 topic</label>
            <input v-model="camera2Topic" class="input is-small" />
          </div>
          <div class="field">
            <label>Camera 2 type</label>
            <input v-model="camera2Type" class="input is-small" />
          </div>
        </div>
      </div>
    </div>
  </div>
</template>

<script>
import { mapState, mapActions } from 'vuex'

const camera1 = 'camera1'
const camera2 = 'camera2'

export default {
  name: 'TeleopConfig',
  computed: {
    ...mapState('teleop', {
      [camera1]: state => state[camera1],
      [camera2]: state => state[camera2]
    }),
    camera1Topic: {
      get() {
        return this.camera1.topic
      },
      set(value) {
        this.setTopic({ cameraName: camera1, topic: value })
      }
    },
    camera1Type: {
      get() {
        return this.camera1.type
      },
      set(value) {
        this.setType({ cameraName: camera1, type: value })
      }
    },
    camera2Topic: {
      get() {
        return this.camera2.topic
      },
      set(value) {
        this.setTopic({ cameraName: camera2, topic: value })
      }
    },
    camera2Type: {
      get() {
        return this.camera2.type
      },
      set(value) {
        this.setType({ cameraName: camera2, type: value })
      }
    }
  },
  methods: {
    ...mapActions('teleop', ['setTopic', 'setType'])
  }
}
</script>

<style lang="scss" scoped>
.teleop-settings {
  display: grid;
  grid-template-rows: auto auto;
  .camera-container {
    display: grid;
    grid-template-columns: 1fr 1fr;
  }
}
</style>
