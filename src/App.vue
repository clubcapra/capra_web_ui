<template>
  <div id="container">
    <navbar />
    <div id="view">
      <router-view />
    </div>
    <takin-footer />
  </div>
</template>

<script lang="ts">
import 'reflect-metadata'
import { Vue, Component, Prop, Provide, Inject } from 'vue-property-decorator'

import Navbar from '@/components/UI/layout/Navbar.vue'
import TakinFooter from '@/components/UI/layout/Footer.vue'

import GamepadManager from '@/utils/gamepad/GamepadManager'
import RosClient from '@/utils/ros/RosClient.ts'
import RosModule from '@/store/modules/ros'

@Component({
  components: {
    Navbar,
    TakinFooter,
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
$footer-height: 30px;

html {
  overflow-y: hidden !important;
}

body {
  display: grid;
  align-content: stretch;
}

#container {
  display: grid;
  grid-template-rows: auto 1fr $footer-height;
  height: 100vh;
  overflow: auto;

  #view {
    display: grid;
    align-content: stretch;
    overflow: auto;
  }
}
</style>
