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
import { Dashboard, Camera, Map2D } from '@/components'
import Viewer3D from '@/components/ros/Viewer3D.vue'
import { cameraModule, teleopModule } from '@/store'

@Component({ components: { Camera, Dashboard, Map2D, Viewer3D } })
export default class Teleop extends Vue {
  get leftCamera() {
    console.log(this.getCamera(teleopModule.leftCamera))
    return cameraModule.cameras[teleopModule.leftCamera]
  }

  get rightCamera() {
    return cameraModule.cameras[teleopModule.rightCamera]
  }

  getCamera(cameraName: string) {
    return cameraModule.getCamera({ cameraName })
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
