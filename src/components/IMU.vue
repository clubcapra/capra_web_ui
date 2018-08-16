<template>
  <div id="IMU">
    <h3>IMU</h3>
    <div>x: {{ orientation.x.toFixed(4) }}</div>
    <div>y: {{ orientation.y.toFixed(4) }}</div>
    <div>z: {{ orientation.z.toFixed(4) }}</div>
  </div>
</template>

<script>
import { mapState } from 'vuex'

export default {
  name: 'IMU',
  computed: mapState({
    connected: state => state.ros.connected,
    error: state => state.ros.error,
    orientation: state => state.ros.orientation
  })
}
</script>

<style lang="stylus" scoped>
#IMU
  border 1px solid red
</style>
