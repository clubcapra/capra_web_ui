<template>
  <div id="IMU">
    <h3>IMU</h3>
    <div id="data">
      <div>x: {{ orientation.x }}</div>
      <div>y: {{ orientation.y }}</div>
      <div>z: {{ orientation.z }}</div>
      <div>temp:{{ temp }}</div>
      <div>speed: {{ speed }} m/s</div>
    </div>
  </div>
</template>

<script>
import { mapState } from 'vuex'
import _ from 'lodash'

let mapDirection = dir => dir.toFixed(4).padStart(6)

export default {
  name: 'IMU',
  data: function() {
    return { speed: 10 }
  },
  computed: mapState('ROS', {
    orientation: state => {
      const { orientation } = state
      let temp = _.mapValues(orientation, mapDirection)
      return temp
    },
    temp: state => state.temperature.toFixed(2)
  }),
  mounted: function() {
    this.move()
  },
  methods: {
    move: function() {
      setInterval(() => {
        const newSpeed = Math.floor(Math.random() * 10 + 1)
        this.speed = newSpeed
      }, 250)
    }
  }
}
</script>

<style lang="scss" scoped>
#IMU {
  #data {
    font-family: monospace;
    line-height: 1em;
  }
}
</style>
