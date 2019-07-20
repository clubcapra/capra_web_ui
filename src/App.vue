<template>
  <div id="container">
    <tabs />
    <div id="view">
      <router-view />
      <status-bar class="status-bar" />
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
import StatusBar from '@/components/ui/layout/StatusBar/StatusBar.vue'
import { rosModule } from '@/store'
import EStop from '@/components/EStop.vue'

import { rosClient } from '@/utils/ros/rosClient'

@Component({
  components: {
    Tabs,
    StatusBar,
    EStop,
  },
})
export default class App extends Vue {
  created() {
    //TODO init in rosModule
    rosClient.setListeners(
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
    .status-bar {
      grid-area: f;
    }
    .right-sidebar {
      grid-area: e;
    }
  }
}
</style>
