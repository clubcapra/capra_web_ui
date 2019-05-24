<template>
  <b-section>
    <b-title>Teleop</b-title>
    <hr />
    <h3>Left camera</h3>
    <b-select is-small>
      <select v-model="leftCamera">
        <option
          v-for="camera in cameras"
          :key="camera.name"
          :value="camera.name"
          >{{ camera.name }}</option
        >
      </select>
    </b-select>

    <h3>Right camera</h3>
    <b-select is-small>
      <select v-model="rightCamera">
        <option
          v-for="camera in cameras"
          :key="camera.name"
          :value="camera.name"
          >{{ camera.name }}</option
        >
      </select>
    </b-select>
  </b-section>
</template>

<script lang="ts">
import { Vue, Component } from 'vue-property-decorator'
import { teleopModule, cameraModule } from '@/store'
import { CameraMap } from '@/store/modules/camera.types'

@Component
export default class TeleopConfig extends Vue {
  cameras = cameraModule.camerasForSelect

  get leftCamera(): string {
    return teleopModule.leftCamera
  }

  set leftCamera(value) {
    teleopModule.setLeftCamera(value)
  }

  rightCamera: string = teleopModule.rightCamera
}
</script>

<style lang="scss" scoped></style>
