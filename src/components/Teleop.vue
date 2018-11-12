<template>
  <div class="teleop">
    <div class="main-view">
      <Camera path="front.rgb" />
      <Camera path="front.depth" />
    </div>
    <div class="bottom-panel">
      <IMU />
      <Map2D />
      <div>
        <!-- <img src="http://robotwebtools.org/images/demos/ros3djs-fetch-urdf.jpg"> -->
      </div>
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
shadow-box = inset 0 0 0 1px #000000

.teleop
  display grid
  align-content stretch
  grid-template-rows 7fr 3fr

  .main-view
    display grid
    grid-template-columns 1fr 1fr

    >div
      box-shadow shadow-box

  .bottom-panel
    display grid
    grid-template-columns 1fr 1fr 1fr

    >div
      box-shadow shadow-box
</style>
