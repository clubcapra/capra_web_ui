<template>
  <div id="dashboard">
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
import { mapState, mapActions } from 'vuex'
import _ from 'lodash'

export default {
  name: 'Dashboard',
  inject: ['rosClient'],
  data: function() {
    return { speed: 10 }
  },
  computed: mapState('dashboard', {
    orientation: state => {
      const orientation = state.orientation.data
      const mapDirection = dir => dir.toFixed(4).padStart(6)
      const temp = _.mapValues(orientation, mapDirection)
      return temp
    },
    temp: state => state.temperature.data.toFixed(2),
    orientationTopic: state => state.orientation.topic,
    temperatureTopic: state => state.temperature.topic
  }),
  mounted: function() {
    this.rosClient.subscribe(this.orientationTopic, this.updateOrientation)
    this.rosClient.subscribe(this.temperatureTopic, this.updateTemperature)

    // give random speed values
    setInterval(() => {
      const newSpeed = Math.floor(Math.random() * 10 + 1)
      this.speed = newSpeed
    }, 250)
  },
  methods: {
    ...mapActions('Dashboard', {
      updateOrientation: 'updateOrientation',
      updateTemperature: 'updateTemperature'
    })
  }
}
</script>

<style lang="scss" scoped>
#dashboard {
  #data {
    font-family: monospace;
    line-height: 1em;
  }
}
</style>
