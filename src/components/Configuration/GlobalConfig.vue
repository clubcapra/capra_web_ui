<template>
  <div class="global-config">
    <ros />
    <hr />
    <teleop />
    <hr />
    <camera />
  </div>
</template>

<script>
import Ros from './RosConfig'
import Teleop from './TeleopConfig'
import Camera from './CameraConfig'

export default {
  name: 'GlobalConfig',
  inject: ['rosClient'],
  components: {
    Ros,
    Teleop,
    Camera,
  },
}
</script>

<style lang="scss" scoped>
.global-config {
  overflow-y: auto;
}
</style>
