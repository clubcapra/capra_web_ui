<template>
  <div class="status-icon">
    <font-awesome-icon :icon="getIcon" />
  </div>
</template>

<script lang="ts">
import { Vue, Component, Inject } from 'vue-property-decorator'
import { FontAwesomeIcon } from '@fortawesome/vue-fontawesome'
import { faTruckMonster, faHandPaper } from '@fortawesome/free-solid-svg-icons'
import { gamepadModule } from '@/store'

@Component({ components: { FontAwesomeIcon } })
export default class GamePadStateInfo extends Vue {
  get getIcon() {
    return gamepadModule.isArmControlled ? faHandPaper : faTruckMonster
  }
}
</script>

<style lang="scss">
.status-icon {
  font-size: 16px;
}
</style>
