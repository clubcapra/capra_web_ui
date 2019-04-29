<template>
  <div />
</template>

<script lang="ts">
import { Vue, Component } from 'vue-property-decorator'

import debug from '@/utils/gamepad/gamepadDebugger.ts'

@Component
export default class GamepadDebugger extends Vue {
  mounted() {
    if (this.$store.state.isDebug) {
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
