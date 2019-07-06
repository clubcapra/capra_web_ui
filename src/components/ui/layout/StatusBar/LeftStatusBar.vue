<template>
  <b-level-left>
    <b-level-item class="rosStatus">{{ rosStatus }}</b-level-item>
  </b-level-left>
</template>

<script lang="ts">
import { Vue, Component } from 'vue-property-decorator'
import { rosModule } from '@/store'

@Component
export default class LeftStatusBar extends Vue {
  get rosStatus() {
    if (rosModule.connecting)
      return 'trying to connect to : ' + rosModule.robotIP
    else if (rosModule.connected) return 'connected to ' + rosModule.url

    return 'disconnected'
  }
}
</script>

<style lang="scss">
.rosStatus {
  color: black;
  font-weight: bold;
}
</style>
