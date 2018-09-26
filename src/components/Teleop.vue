<template>
  <div class="teleop">
    <div class="main-view">
      <Camera path="front.rgb" />
      <Camera path="front.depth" />
    </div>
    <div class="bottom-panel">
      <IMU />
      <Map2D />
      <div />
    </div>
  </div>
</template>

<script>
import IMU from '@/components/IMU'
import Camera from '@/components/Camera'
import Map2D from '@/components/Map2D'

export default {
  name: 'Teleop',
  components: {
    IMU,
    Camera,
    Map2D
  }
}
</script>

<style lang="stylus">
.teleop
  display grid
  height 100%
  min-height 0
  grid-template-rows 50% 50%

  .main-view
    display grid
    grid-template-columns 50% 50%

  .bottom-panel
    display grid
    grid-template-columns 1fr 1fr 1fr
</style>
