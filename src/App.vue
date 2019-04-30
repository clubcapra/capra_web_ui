<template>
  <div id="container">
    <navbar />
    <div id="view">
      <router-view />
    </div>
    <gamepad-debugger />
    <takin-footer />
  </div>
</template>

<script lang="ts">
import 'reflect-metadata'
import { Vue, Component, Prop, Provide, Inject } from 'vue-property-decorator'

import Navbar from '@/components/Navbar.vue'
import TakinFooter from '@/components/Footer.vue'
import GamepadDebugger from '@/components/GamepadDebugger.vue'

import GamepadManager from '@/utils/gamepad/GamepadManager'
import RosClient from '@/utils/ros/RosClient.ts'
import RosModule from '@/store/modules/ros'

@Component({
  components: {
    Navbar,
    GamepadDebugger,
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
@import '~bulma/sass/utilities/_all';
@import '~bulmaswatch/slate/_variables.scss';
@import '~bulma';
@import '~bulmaswatch/slate/_overrides.scss';

html {
  height: 100%;
  overflow-y: auto;
}

body {
  min-height: 100%;
  display: grid;
  align-content: stretch;
}

#container {
  display: grid;
  grid-template-rows: auto 1fr;
  overflow: auto;

  #view {
    display: grid;
    align-content: stretch;
    overflow: auto;
  }
}
</style>
