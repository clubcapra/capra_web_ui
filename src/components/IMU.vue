<template>
  <div id="IMU">
    <h3>IMU</h3>
    <pre>{{ twist }}</pre>
    <pre>{{ pose }}</pre>
  </div>
</template>

<script>
import { mapState } from 'vuex'

export default {
  name: 'IMU',
  computed: mapState({
    connected: state => state.ros.connected,
    error: state => state.ros.error,
    twist: state => state.ros.twist,
    pose: state => state.ros.pose
  })
}
</script>

<style lang="stylus" scoped>
#IMU
  border 1px solid red
</style>
