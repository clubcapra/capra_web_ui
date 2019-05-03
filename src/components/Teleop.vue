<template>
  <div class="teleop">
    <div class="main-view">
      <Camera :type="camera3drgb.type" :topic="camera3drgb.topic" />
      <Camera :type="camera3ddepth.type" :topic="camera3ddepth.topic" />
    </div>
    <div class="bottom-panel">
      <Dashboard />
      <Map2D />
      <div>
        <img
          src="http://robotwebtools.org/images/demos/ros3djs-fetch-urdf.jpg"
        />
      </div>
    </div>
  </div>
</template>

<script lang="ts">
import { Vue, Component, Prop, Inject } from 'vue-property-decorator'

import Dashboard from '@/components/Dashboard.vue'
import Camera from '@/components/Camera.vue'
import Map2D from '@/components/Map2D.vue'

import CameraModule from '@/store/modules/camera'

@Component({ components: { Camera, Dashboard, Map2D } })
export default class Teleop extends Vue {
  get camera3drgb() {
    return {
      type: CameraModule.cameras.camera3d_rgb.type,
      topic: CameraModule.cameras.camera3d_rgb.topic,
    }
  }

  get camera3ddepth() {
    return {
      type: CameraModule.cameras.camera3d_depth.type,
      topic: CameraModule.cameras.camera3d_depth.topic,
    }
  }
}
</script>

<style lang="scss">
.teleop {
  display: grid;
  align-content: stretch;
  grid-template-rows: 70% 30%;

  .main-view {
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
