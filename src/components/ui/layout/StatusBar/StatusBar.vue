<template>
  <div :class="`status-bar ${backgroundColour}`">
    <div>{{ rosStatus }}</div>
    <div />
    <div />
    <div>
      <NetworkInfo />
    </div>
    <div>{{ currentTime }}</div>
  </div>
</template>

<script>
import { Vue, Component } from 'vue-property-decorator'
import { clearInterval } from 'timers'
import { rosModule } from '@/store'
import NetworkInfo from './NetworkInfo.vue'

@Component({ components: { NetworkInfo } })
export default class StatusBar extends Vue {
  currentTime = new Date().toLocaleTimeString()

  get rosStatus() {
    if (rosModule.connecting)
      return 'trying to connect to : ' + rosModule.robotIP
    else if (rosModule.connected) return 'connected to ' + rosModule.url

    return 'disconnected'
  }

  get backgroundColour() {
    if (rosModule.connecting) return 'has-background-warning has-text-black'
    return this.connected ? 'has-background-success' : 'has-background-danger'
  }

  mounted() {
    this.interval = setInterval(this.updateTime, 1000)
  }

  updateTime() {
    this.currentTime = new Date().toLocaleTimeString()
  }

  destroyed() {
    clearInterval(this.interval)
  }
}
</script>

<style lang="scss">
$Padding-offset: 0.5%;

.status-bar {
  // display: grid;
  // grid-template-columns: auto auto auto auto auto;
  //border-top: 1px solid black;
  height: 100%;
  font-size: 0.75em;
  background-color: purple;
  display: flex;
  flex-wrap: nowrap;
  justify-content: space-between;
  padding-left: $Padding-offset;
  padding-right: $Padding-offset;
}
</style>
