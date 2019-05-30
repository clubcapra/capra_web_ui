<template>
  <div id="container">
    <tabs />
    <div id="view">
      <router-view />
      <takin-footer class="takin-footer" />
      <div class="right-sidebar">
        <e-stop />
      </div>
    </div>
  </div>
</template>

<script lang="ts">
import 'reflect-metadata'
import { Vue, Component, Prop, Provide, Inject } from 'vue-property-decorator'

import Tabs from '@/components/ui/layout/Tabs.vue'
import TakinFooter from '@/components/ui/layout/Footer.vue'

import GamepadManager from '@/utils/gamepad/GamepadManager'
import RosClient from '@/utils/ros/RosClient.ts'
import { rosModule } from '@/store'
import EStop from '@/components/EStop.vue'

@Component({
  components: {
    Tabs,
    TakinFooter,
    EStop,
  },
})
export default class App extends Vue {
  @Provide() gamepadManager = new GamepadManager()

  created() {
    //TODO init in rosModule
    RosClient.setListeners(
      rosModule.onConnect,
      rosModule.onDisconnect,
      () => {}
    )

    rosModule.connect()
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
    overflow: auto;

    display: grid;
    grid-template-areas:
      'v e'
      'f e';
    grid-template-columns: auto $e-stop-width;
    grid-template-rows: auto 20px;

    &:first-child {
      grid-area: v;
    }
    .takin-footer {
      grid-area: f;
    }
    .right-sidebar {
      grid-area: e;
    }
  }
}
</style>
