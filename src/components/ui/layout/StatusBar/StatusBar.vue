<template>
  <b-level :class="`status-bar ${backgroundColour}`">
    <b-level-left>
      <left-status-bar />
    </b-level-left>
    <right-status-bar />
  </b-level>
</template>

<script>
import { Vue, Component } from 'vue-property-decorator'
import { rosModule } from '@/store'
import LeftStatusBar from './LeftStatusBar'
import RightStatusBar from './RightStatusBar'

@Component({ components: { LeftStatusBar, RightStatusBar } })
export default class StatusBar extends Vue {
  get backgroundColour() {
    if (rosModule.connecting) return 'has-background-warning has-text-black'

    return rosModule.connected
      ? 'has-background-success'
      : 'has-background-danger'
  }
}
</script>

<style lang="scss">
$Padding-offset: 0.5%;

.status-bar {
  height: 100%;
  font-size: 0.75em;
  margin: 0px !important;
  padding-left: $Padding-offset;
  padding-right: $Padding-offset;
}
</style>
