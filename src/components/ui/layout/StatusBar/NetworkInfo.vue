<template>
  <div class="netInfo">
    <!-- <i class="material-icons" style="font-size: 16px">{{ networkIcon }}</i> -->
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
    this.networkIcon = this.getNetworkIcon(connection)
    this.networkRTT = connection.rtt
  }

  getNetworkIcon(connection: any) {
    switch (connection.type) {
      case 'bluetooth':
        switch (connection.effectiveType) {
          case 'slow-2g':
          case '2g':
          case '3g':
          case '4g':
            return 'bluetooth_connected'
          default:
            return 'bluetooth_disabled'
        }
      case 'cellular':
        switch (connection.effectiveType) {
          case 'slow-2g':
            return 'signal_cellular_1_bar'
          case '2g':
            return 'signal_cellular_2_bar'
          case '3g':
            return 'signal_cellular_3_bar'
          case '4g':
            return 'signal_cellular_4_bar'
          default:
            return 'signal_cellular_off'
        }
      case 'ethernet':
        return 'settings_ethernet'
      case 'wifi':
      default:
        switch (connection.effectiveType) {
          case 'slow-2g':
            return 'signal_wifi_1_bar'
          case '2g':
            return 'signal_wifi_2_bar'
          case '3g':
            return 'signal_wifi_3_bar'
          case '4g':
            return 'signal_wifi_4_bar'
          default:
            return 'signal_wifi_off'
        }
    }
  }

  logNetworkInfo() {
    /* eslint-disable no-console */

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

    /* eslint-enable no-console */
  }
}
</script>
