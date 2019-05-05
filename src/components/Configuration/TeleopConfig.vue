<template>
  <b-section>
    <b-title>Teleop</b-title>
    <hr />
  </b-section>
</template>

<script lang="ts">
import { Vue, Component } from 'vue-property-decorator'

@Component
export default class TeleopConfig extends Vue {
  camera1: string = 'camera1'
  camera2: string = 'camera2'
}
</script>

<style lang="scss" scoped></style>
