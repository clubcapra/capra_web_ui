<template>
  <div class="section teleop-settings"><p class="title">Teleop</p></div>
</template>

<script>
export default {
  name: 'TeleopConfig',
  data() {
    return {
      camera1: 'camera1',
      camera2: 'camera2'
    }
  }
}
</script>

<style lang="scss" scoped>
.teleop-settings {
  display: grid;
  grid-template-rows: auto auto;
  .camera-container {
    display: grid;
    grid-template-columns: 1fr 1fr;
  }
}
</style>
