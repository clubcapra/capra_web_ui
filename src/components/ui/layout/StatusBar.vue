<template>
  <footer class="takin-footer">
    <div>allo</div>
    <div>allo2</div>
    <div>allo2</div>
    <div><span class="icon"><i class="material-icons" v-text="networkIcon"></i></span>allo2</div>
    <div v-text="currentTime"></div>
  </footer>
</template>

<script>
import { Vue, Component } from 'vue-property-decorator'
import { clearInterval } from 'timers';

@Component
export default class StatusBar extends Vue {
  currentTime = (new Date()).toLocaleTimeString()
  networkRTT = navigator.connection.rtt
  networkEffectiveType = navigator.connection.effectiveType
  networkIcon = "signal_wifi_off"

  mounted(){
    this.interval = setInterval(this.updateTime, 1000);
    this.networkInfo = setInterval(this.logNetworkInfo, 1000);
  }

  updateTime() {
    this.currentTime = (new Date()).toLocaleTimeString()
  }

  logNetworkInfo() {
    // Network type that browser uses
    console.log('         type: ' + navigator.connection.type);
    switch (navigator.connection.type) {
      case "bluetooth" :
        switch (navigator.connection.effectiveType) {
          case "slow-2g":
          case "2g":
          case "3g":
          case "4g":
            this.networkIcon="bluetooth_connected"
            break;

          default:
            this.networkIcon="bluetooth_disabled"
            break;
        }
        break;
    case "cellular" :
        switch (navigator.connection.effectiveType) {
          case "slow-2g":
            this.networkIcon="signal_cellular_1_bar"
            break;
          case "2g":
            this.networkIcon="signal_cellular_2_bar"
            break;
          case "3g":
            this.networkIcon="signal_cellular_3_bar"
            break;
          case "4g":
            this.networkIcon="signal_cellular_4_bar"
            break;

          default:
            this.networkIcon="signal_cellular_off"
            break;
        }
        break;
    case "ethernet" :
        this.networkIcon = "settings_ethernet"
        break;
    case "wifi" :
      default:
        switch (navigator.connection.effectiveType) {
          case "slow-2g":
            this.networkIcon = "signal_wifi_1_bar"
            break;
          case "2g":
            this.networkIcon = "signal_wifi_2_bar"
            break;
          case "3g":
            this.networkIcon = "signal_wifi_3_bar"
            break;
          case "4g":
            this.networkIcon = "signal_wifi_4_bar"
            break;

          default:
            this.networkIcon = "signal_wifi_off"
            break;
        }
        // Set wifi icon as default
        break;
    }

    this.networkRTT = navigator.connection.rtt
    this.networkEffectiveType = navigator.connection.effectiveType



    // Effective bandwidth estimate
    console.log('     downlink: ' + navigator.connection.downlink + 'Mb/s');

    // Effective round-trip time estimate
    console.log('          rtt: ' + navigator.connection.rtt + 'ms');

    // Upper bound on the downlink speed of the first network hop
    console.log('  downlinkMax: ' + navigator.connection.downlinkMax + 'Mb/s');

    // Effective connection type determined using a combination of recently
    // observed rtt and downlink values: ' +
    console.log('effectiveType: ' + navigator.connection.effectiveType);

    // True if the user has requested a reduced data usage mode from the user
    // agent.
    console.log('     saveData: ' + navigator.connection.saveData);
  }

  destroyed() {
    clearInterval(this.interval)
    clearInterval(this.networkInfo)
  }

}
</script>

<style lang="scss">
$Padding-offset: 0.5%;

.takin-footer {
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
