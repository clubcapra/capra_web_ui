<template>
  <div />
</template>

<script lang="ts">
import { Vue, Component } from 'vue-property-decorator'

import padDebugger from '@/utils/gamepad/debugger/GamepadDebugger.ts'

@Component
export default class GamepadDebugger extends Vue {
  gamepadDebugger: padDebugger = new padDebugger()
}
</script>

<style lang="scss">
.buttons {
  span.button.pressed {
    background-color: red !important;
  }
}
</style>
