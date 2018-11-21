<template>
  <card :title="title">
    <div class="field">
      <label>Topic</label> <input v-model="topic" class="input is-small" />
    </div>
    <div class="field">
      <label>Type</label> <input v-model="type" class="input is-small" />
    </div>
  </card>
</template>

<script>
import { mapActions } from 'vuex'
import Card from '../Card/Card'

export default {
  name: 'CameraCard',
  components: { Card },
  props: {
    title: {
      type: String,
      default: ''
    },
    cameraName: {
      type: String,
      default: ''
    }
  },
  computed: {
    camera() {
      return this.$store.state.camera[this.cameraName]
    },
    topic: {
      get() {
        return this.camera.topic
      },
      set(value) {
        console.log('​set -> value', value)
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
    ...mapActions('camera', ['setTopic', 'setType'])
  }
}
</script>
