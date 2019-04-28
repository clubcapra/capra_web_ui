<template>
  <div />
</template>

<script lang="ts">
import { Vue, Component } from 'vue-property-decorator'
import debug from '@/gamepad/gamepadDebugger.ts'
import GamepadManager from '@/gamepad/GamepadManager.ts'
import CustomGamepad from '@/gamepad/CustomGamepad.ts'

const ifdebug = true

@Component
export default class GamepadDebugger extends Vue {
  mounted() {
    if (ifdebug) {
      debug()
    }
  }
}
</script>

<style lang="scss">
.buttons {
  span.button.pressed {
    background-color: red !important;
  }
}
</style>
