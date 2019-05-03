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

import Navbar from '@/components/Navbar.vue'
import TakinFooter from '@/components/Footer.vue'

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
@import '~bulma/sass/utilities/_all';

$primary: red;
$link: $primary;

$body-background-color: $dark;
$background: #ffffff;
$card-background-color: $grey-dark;
$text: $grey-lighter;
$text-strong: $text;
$text-light: $text;
$title-color: $text;
$label-color: $text;

@import '~bulma';

html {
  height: 100%;
  overflow-y: auto;
}

body {
  min-height: 100%;
  display: grid;
  align-content: stretch;
  // background: $red;
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
