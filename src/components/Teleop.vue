<template>
  <div class="teleop">
    <div class="cameras">
      <camera :type="leftCamera.type" :topic="leftCamera.topic" />
      <camera :type="rightCamera.type" :topic="rightCamera.topic" />
    </div>
    <div class="bottom-panel">
      <dashboard />
      <Map2D />
      <Viewer3D />
    </div>
  </div>
</template>

<script lang="ts">
import { Vue, Component } from 'vue-property-decorator'

import Dashboard from '@/components/Dashboard.vue'
import Camera from '@/components/Camera.vue'
import Map2D from '@/components/Map2D.vue'
import Viewer3D from '@/components/ros/Viewer3D.vue'

import CameraModule from '@/store/modules/camera'
import TeleopModule from '@/store/modules/teleop'

@Component({ components: { Camera, Dashboard, Map2D, Viewer3D } })
export default class Teleop extends Vue {
  get leftCamera() {
    return CameraModule.cameras[TeleopModule.leftCamera]
  }

  get rightCamera() {
    return CameraModule.cameras[TeleopModule.rightCamera]
  }
}
</script>

<style lang="scss">
.teleop {
  display: grid;
  align-content: stretch;
  grid-template-rows: 70% 30%;

  .cameras {
    display: grid;
    grid-template-columns: 1fr 1fr;

    > div {
      box-shadow: inset 0 0 0 0.5px #000000;
    }
  }

  .bottom-panel {
    display: grid;
    grid-template-columns: 1fr 1fr 1fr;

    > div {
      box-shadow: inset 0 0 0 0.5px #000000;
    }
  }
}
</style>
