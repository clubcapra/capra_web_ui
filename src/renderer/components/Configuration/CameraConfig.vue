<template>
  <div class="card">
    <div class="card-header">
      <p class="card-header-title">{{ title }}</p>
    </div>
    <div class="card-content">
      <div class="field">
        <label>Topic</label> <input v-model="topic" class="input is-small" />
      </div>
      <div class="field">
        <label>Type</label> <input v-model="type" class="input is-small" />
      </div>
    </div>
  </div>
</template>

<script>
import { mapActions, mapState } from 'vuex'

export default {
  name: 'CameraConfig',
  props: {
    title: {
      type: String,
      default: ''
    },
    cameraName: {
      type: String,
      default: ''
    },
    moduleName: {
      type: String,
      default: ''
    }
  },
  computed: {
    ...mapState({
      camera: state => state[this.moduleName][this.cameraName]
    }),
    topic: {
      get() {
        return this.camera.topic
      },
      set(value) {
        this.setTopic({ cameraName: this.cameraName, topic: value })
      }
    },
    type: {
      get() {
        return this.camera.type
      },
      set(value) {
        this.setType({ cameraName: this.cameraName, type: value })
      }
    }
  },
  methods: {
    ...mapActions('teleop', ['setTopic', 'setType'])
  }
}
</script>
