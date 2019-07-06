<template>
  <div class="icon">
    <font-awesome-icon :icon="getIcon" />
  </div>
</template>

<script lang="ts">
import { Vue, Component, Inject } from 'vue-property-decorator'
import { extend } from 'lodash-es'
import GamepadManager from '@/utils/gamepad/GamepadManager'
import { FontAwesomeIcon } from '@fortawesome/vue-fontawesome'
import { faTruckMonster, faHandPaper } from '@fortawesome/free-solid-svg-icons'
import { library } from '@fortawesome/fontawesome-svg-core'

@Component({ components: { FontAwesomeIcon } })
export default class GamePadStateInfo extends Vue {
  @Inject() gamepadManager!: GamepadManager

  get gamePadStateInfo() {
    return this.gamepadManager.getIsArmControlled
  }

  get getIcon() {
    return this.gamePadStateInfo ? faHandPaper : faTruckMonster
  }
}
</script>

<style lang="scss">
.icon {
  color: black;
  font-weight: bold;
  height: 100%;
  font-size: 1.05rem;
}
</style>

