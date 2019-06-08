<template>
  <div>
    <i class="material-icons" style="font-size: 16px">{{ networkIcon }}</i>
    {{ networkRTT }}ms
  </div>
</template>

<script lang="ts">
import { Vue, Component } from 'vue-property-decorator'
import { clearInterval } from 'timers'

@Component
export default class NetworkInfo extends Vue {
  // @ts-ignore
  networkRTT = navigator.connection.rtt
  // @ts-ignore
  networkEffectiveType = navigator.connection.effectiveType
  networkIcon = 'signal_wifi_off'
  networkInfo!: NodeJS.Timeout

  mounted() {
    this.networkInfo = setInterval(this.updateNetworkInfo, 1000)
  }

  destroyed() {
    clearInterval(this.networkInfo)
  }

  updateNetworkInfo() {
    // logNetworkInfo()
    // @ts-ignore
    const connection: any = navigator.connection
    switch (connection.type) {
      case 'bluetooth':
        switch (connection.effectiveType) {
          case 'slow-2g':
          case '2g':
          case '3g':
          case '4g':
            this.networkIcon = 'bluetooth_connected'
            break

          default:
            this.networkIcon = 'bluetooth_disabled'
            break
        }
        break
      case 'cellular':
        switch (connection.effectiveType) {
          case 'slow-2g':
            this.networkIcon = 'signal_cellular_1_bar'
            break
          case '2g':
            this.networkIcon = 'signal_cellular_2_bar'
            break
          case '3g':
            this.networkIcon = 'signal_cellular_3_bar'
            break
          case '4g':
            this.networkIcon = 'signal_cellular_4_bar'
            break

          default:
            this.networkIcon = 'signal_cellular_off'
            break
        }
        break
      case 'ethernet':
        this.networkIcon = 'settings_ethernet'
        break
      case 'wifi':
      default:
        switch (connection.effectiveType) {
          case 'slow-2g':
            this.networkIcon = 'signal_wifi_1_bar'
            break
          case '2g':
            this.networkIcon = 'signal_wifi_2_bar'
            break
          case '3g':
            this.networkIcon = 'signal_wifi_3_bar'
            break
          case '4g':
            this.networkIcon = 'signal_wifi_4_bar'
            break
          default:
            this.networkIcon = 'signal_wifi_off'
            break
        }
        break
    }

    this.networkRTT = connection.rtt
    this.networkEffectiveType = connection.effectiveType
  }

  logNetworkInfo() {
    // @ts-ignore
    const connection: any = navigator.connection

    // Network type that browser uses
    console.log('         type: ' + connection.type)

    // Effective bandwidth estimate
    console.log('     downlink: ' + connection.downlink + 'Mb/s')

    // Effective round-trip time estimate
    console.log('          rtt: ' + connection.rtt + 'ms')

    // Upper bound on the downlink speed of the first network hop
    console.log('  downlinkMax: ' + connection.downlinkMax + 'Mb/s')

    // Effective connection type determined using a combination of recently
    // observed rtt and downlink values: ' +
    console.log('effectiveType: ' + connection.effectiveType)

    // True if the user has requested a reduced data usage mode from the user
    // agent.
    console.log('     saveData: ' + connection.saveData)
  }
}
</script>
