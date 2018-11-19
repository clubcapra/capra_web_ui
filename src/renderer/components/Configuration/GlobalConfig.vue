<template>
  <div id="settings">
    <ros />
    <hr />
    <teleop />
  </div>
</template>

<script>
import Ros from './RosConfig'
import Teleop from './TeleopConfig'

export default {
  name: 'GlobalConfig',
  inject: ['rosClient'],
  components: {
    Ros,
    Teleop
  }
}
</script>

<style lang="scss" scoped>
#settings {
  overflow-y: auto;
}
</style>
