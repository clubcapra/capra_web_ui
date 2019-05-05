<template>
  <b-section>
    <b-title>Gamepad</b-title>
    <hr />
  </b-section>
</template>

<script lang="ts">
import { Vue, Component } from 'vue-property-decorator'

@Component
export default class GamepadConfig extends Vue {
  temp = 1
}
</script>

<style lang="scss" scoped></style>
