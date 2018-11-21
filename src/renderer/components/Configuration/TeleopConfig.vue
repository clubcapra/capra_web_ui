<template>
  <div class="section teleop-settings">
    <p class="title">Teleop</p>
    <div class="camera-container">
      <camera-config
        title="Camera 1"
        :camera-name="camera1"
        :module-name="moduleName"
      />
      <camera-config
        title="Camera 2"
        :camera-name="camera2"
        :module-name="moduleName"
      />
    </div>
  </div>
</template>

<script>
import CameraConfig from './CameraConfig'

export default {
  name: 'TeleopConfig',
  components: { CameraConfig },
  data() {
    return { camera1: 'camera1', camera2: 'camera2', moduleName: 'teleop' }
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
