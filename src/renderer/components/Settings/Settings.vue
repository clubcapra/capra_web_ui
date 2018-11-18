<template>
  <div id="settings">
    <ros-settings />
    <hr />
    <teleop-settings />
  </div>
</template>

<script>
import RosSettings from './RosSettings'
import TeleopSettings from './TeleopSettings'

export default {
  name: 'Settings',
  inject: ['rosClient'],
  components: {
    RosSettings,
    TeleopSettings
  }
}
</script>

<style lang="scss" scoped>
#settings {
  overflow-y: auto;
}
</style>
