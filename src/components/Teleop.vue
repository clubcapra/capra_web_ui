<template>
  <div id="teleop">
    <h3>Teleop</h3>
    <div class="top-panel"/>
    <div class="bottom-panel">
      <IMU />
      <div class="panel-1"/>
      <div class="panel-2"/>
    </div>
  </div>
</template>

<script>
import IMU from '@/components/IMU'
export default {
  name: 'Teleop',
  components: {
    IMU
  }
}
</script>

<style lang="stylus" scoped>
#teleop
  display grid
  height 100%

  .bottom-panel
    height 50%

  .bottom-panel
    height 50%
    display grid
    grid-template-columns 1fr 1fr 1fr
</style>
