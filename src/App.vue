<template>
  <div id="container">
    <navbar />
    <div id="view">
      <router-view />
    </div>
    <gamepad-debugger />
  </div>
</template>

<script lang="ts">
import 'reflect-metadata'
import { Vue, Component, Prop, Provide, Inject } from 'vue-property-decorator'
import { namespace, Action, State } from 'vuex-class'

import Navbar from '@/components/Navbar.vue'
import GamepadDebugger from '@/components/GamepadDebugger.vue'

import RosClient from '@/RosClient.ts'

import RosModule from '@/store/modules/ros'
import { getModule } from 'vuex-module-decorators'

const rosModule = getModule(RosModule)

@Component({
  components: {
    Navbar,
    GamepadDebugger,
  },
})
export default class App extends Vue {
  @Provide() rosClient = new RosClient()

  created() {
    this.rosClient.setListeners(
      rosModule.onConnect,
      rosModule.onDisconnect,
      () => {}
    )

    this.rosClient.connect(rosModule.robotIP)
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
