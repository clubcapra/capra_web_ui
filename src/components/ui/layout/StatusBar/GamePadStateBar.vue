<template>
  <div>{{ gamePadStateInfo }}</div>
</template>

<script lang="ts">
import { Vue, Component, Inject } from 'vue-property-decorator'
import { extend } from 'lodash-es'
import GamepadManager from '@/utils/gamepad/GamepadManager'

@Component
export default class GamePadStateInfo extends Vue {
  @Inject() gamepadManager!: GamepadManager

  get gamePadStateInfo() {
    const game = this.gamepadManager
    return game.getIsArmControlled
  }
}
</script>
