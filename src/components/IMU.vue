<template lang="pug">
#IMU
  h3 IMU
  div x: {{ orientation.x.toFixed(4) }}
  div y: {{ orientation.y.toFixed(4) }}
  div z: {{ orientation.z.toFixed(4) }}
</template>

<script>
import { mapState } from 'vuex'

export default {
  name: 'IMU',
  computed: mapState({
    orientation: state => state.ros.orientation
  })
}
</script>

<style lang="stylus" scoped>
</style>
