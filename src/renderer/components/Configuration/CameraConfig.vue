<template>
  <div class="section">
    <p class="title">Camera</p>
    <div class="camera-container">
      <camera-card
        v-for="(camera, key) in cameras"
        :key="key"
        :title="key"
        :camera-name="key"
      />
    </div>
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
    justify-content: space-evenly;
  }
}
</style>
