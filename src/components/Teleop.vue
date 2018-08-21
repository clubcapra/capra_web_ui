<template lang="pug">
#teleop
  .columns#main-view
    .column#camera1
      FrontCameraRGB
    .column#camera2
      FrontCamera3d
  .columns.bottom-panel
    .column
      IMU
    .column
    .column
</template>

<script>
import IMU from '@/components/IMU'
import FrontCameraRGB from '@/components/cameras/FrontCameraRGB'
import FrontCamera3d from '@/components/cameras/FrontCamera3d'

export default {
  name: 'Teleop',
  components: {
    IMU,
    FrontCameraRGB,
    FrontCamera3d
  }
}
</script>

<style lang="stylus" scoped>
#teleop
  flex 1 1 auto

  #main-view
    height 65%
</style>
