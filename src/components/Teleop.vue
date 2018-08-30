<template lang="pug">
#teleop
  .columns#main-view
    .column
      Camera(path='front.rgb')
    .column
      Camera(path='front.depth')
  .columns.bottom-panel
    .column
      IMU
    .column
    .column
</template>

<script>
import IMU from '@/components/IMU'
import Camera from '@/components/Camera'

export default {
  name: 'Teleop',
  components: {
    IMU,
    Camera
  }
}
</script>

<style lang="stylus" scoped>
#teleop
  flex 1 1 auto

  #main-view
    height 50vh
</style>
