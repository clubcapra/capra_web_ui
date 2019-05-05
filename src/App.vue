<template>
  <div id="container">
    <tabs />
    <div id="view">
      <router-view />
      <div class="right-sidebar">
        <e-stop />
      </div>
    </div>
    <!-- <takin-footer /> -->
  </div>
</template>

<script lang="ts">
import 'reflect-metadata'
import { Vue, Component, Prop, Provide, Inject } from 'vue-property-decorator'

import Tabs from '@/components/UI/layout/Tabs.vue'
import TakinFooter from '@/components/UI/layout/Footer.vue'

import GamepadManager from '@/utils/gamepad/GamepadManager'
import RosClient from '@/utils/ros/RosClient.ts'
import RosModule from '@/store/modules/ros'
import EStop from '@/components/EStop.vue'

@Component({
  components: {
    Tabs,
    TakinFooter,
    EStop,
  },
})
export default class App extends Vue {
  @Provide() rosClient = new RosClient()
  @Provide() gamepadManager = new GamepadManager(this.rosClient)

  created() {
    this.rosClient.setListeners(
      RosModule.onConnect,
      RosModule.onDisconnect,
      () => {}
    )

    this.rosClient.connect(RosModule.robotIP)
  }
}
</script>

<style lang="scss">
$e-stop-width: 70px;

html {
  overflow-y: hidden !important;
}

body {
  display: grid;
  align-content: stretch;
}

#container {
  display: grid;
  grid-template-rows: auto 1fr;
  height: 100vh;
  overflow: auto;

  #view {
    display: grid;
    grid-template-columns: auto $e-stop-width;
    align-content: stretch;
    overflow: auto;
  }
}
</style>
