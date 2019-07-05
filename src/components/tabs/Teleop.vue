<template>
  <div class="teleop">
    <div class="cameras">
      <camera :type="leftCamera.type" :topic="leftCamera.topic" />
      <camera :type="rightCamera.type" :topic="rightCamera.topic" />
    </div>
    <div class="bottom-panel">
      <dashboard />
      <Map2D />
      <camera :type="bottomCamera.type" :topic="bottomCamera.topic" />
    </div>
  </div>
</template>

<script lang="ts">
import { Vue, Component } from 'vue-property-decorator'
import { Dashboard, Camera, Map2D } from '@/components'
import Viewer3D from '@/components/ros/Viewer3D.vue'
import { cameraModule, teleopModule } from '@/store'

@Component({ components: { Camera, Dashboard } })
export default class Teleop extends Vue {
  get leftCamera() {
    return cameraModule.getCamera(teleopModule.leftCamera)
  }

  get rightCamera() {
    return cameraModule.getCamera(teleopModule.rightCamera)
  }

  get bottomCamera() {
    return cameraModule.getCamera(teleopModule.bottomCamera)
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
