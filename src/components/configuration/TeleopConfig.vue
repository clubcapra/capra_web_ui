<template>
  <b-section>
    <b-title>Teleop</b-title>
    <hr />
  </b-section>
</template>

<script lang="ts">
import { Vue, Component } from 'vue-property-decorator'
import { teleopModule } from '@/store'

@Component
export default class TeleopConfig extends Vue {
  camera1: string = teleopModule.leftCamera
  camera2: string = teleopModule.rightCamera
}
</script>

<style lang="scss" scoped></style>
