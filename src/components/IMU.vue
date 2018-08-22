<template lang="pug">
#IMU
  h3 IMU
  div
    | x: {{ orientation.x }}
    | y: {{ orientation.y }}
    | z: {{ orientation.z }}
    | temp: {{ temp }}
</template>

<script>
import { mapState } from 'vuex'
import _ from 'lodash'

let mapDirection = dir => dir.toFixed(4).padStart(6)

export default {
  name: 'IMU',
  computed: mapState({
    orientation: state => _.mapValues(state.ros.orientation, mapDirection),
    temp: state => state.ros.temperature.toFixed(2)
  })
}
</script>

<style lang="stylus" scoped>
#IMU
  font-family monospace
  white-space pre
</style>
