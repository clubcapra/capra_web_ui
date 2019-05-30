<template>
  <footer class="takin-footer">{{ status }}</footer>
</template>

<script>
import { Vue, Component } from 'vue-property-decorator'
import { rosModule } from '@/store'

@Component
export default class Footer extends Vue {
  get status() {
    if (rosModule.connecting)
      return 'trying to connect to : ' + rosModule.robotIP
    else if (rosModule.connected) return 'connected to ' + rosModule.url

    return 'disconnected'
  }
}
</script>

<style lang="scss">
.takin-footer {
  border-top: 1px solid black;
  height: 100%;
  font-size: 0.75em;
}
</style>
