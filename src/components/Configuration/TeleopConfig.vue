<template>
  <div class="section teleop-settings"><p class="title">Teleop</p></div>
</template>

<script lang="ts">
import { Vue, Component } from 'vue-property-decorator'

@Component
export default class TeleopConfig extends Vue {
  camera1: string = 'camera1'
  camera2: string = 'camera2'
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
