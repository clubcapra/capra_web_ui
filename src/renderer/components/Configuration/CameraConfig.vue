<template>
  <div class="section">
    <p class="title">Camera</p>
    <b-columns class="camera-container">
      <b-column v-for="(camera, key) in cameras" :key="key">
        <camera-card class="camera" :title="key" :camera-name="key" />
      </b-column>
    </b-columns>
  </div>
</template>

<script>
import { mapState } from 'vuex'
import CameraCard from './CameraCard'

export default {
  name: 'CameraConfig',
  components: { CameraCard },
  computed: {
    ...mapState('camera', {
      cameras: state => state
    })
  }
}
</script>

<style lang="scss" scoped>
.section {
  display: grid;
  grid-template-rows: auto auto;
  .camera-container {
    display: grid;
    grid-template-columns: 1fr 1fr;
  }
}
</style>
