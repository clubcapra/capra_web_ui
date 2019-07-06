<template>
  <b-level-right>
    <b-level-item>
      <network-info />
    </b-level-item>
    <b-level-item class="time">{{ currentTime }}</b-level-item>
  </b-level-right>
</template>

<script lang="ts">
import { Vue, Component } from 'vue-property-decorator'
import { rosModule } from '@/store'
import NetworkInfo from './NetworkInfo.vue'

@Component({ components: { NetworkInfo } })
export default class RightStatusBar extends Vue {
  currentTime = new Date().toLocaleTimeString()
  interval: any = null

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
