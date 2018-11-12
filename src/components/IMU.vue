<template>
  <div id="IMU">
    <h3>IMU</h3>
    <div id="data">
      x: {{ orientation.x }}
      y: {{ orientation.y }}
      z: {{ orientation.z }}
      temp: {{ temp }}
      speed: {{ speed }} m/s
    </div>
  </div>
</template>

<script>
import { mapState } from 'vuex'
import _ from 'lodash'

let mapDirection = dir => dir.toFixed(4).padStart(6)

export default {
  name: 'IMU',
  computed: mapState({
    orientation: state => {
      const { orientation } = state.ros
      let temp = _.mapValues(orientation, mapDirection)
      return temp
    },
    temp: state => state.ros.temperature.toFixed(2),
    speed: () => Math.floor(Math.random() * 10 + 1)
  })
}
</script>

<style lang="stylus" scoped>
#IMU
  #data
    font-family monospace
    white-space pre
    line-height 1em
</style>
