<template>
  <b-section>
    <b-title>Victim</b-title>
    <hr />
    <h3>Camera</h3>
    <b-select is-small>
      <select v-model="camera">
        <option v-for="c in cameras" :key="c.name" :value="c.name">{{
          c.name
        }}</option>
      </select>
    </b-select>
  </b-section>
</template>

<script lang="ts">
import { Vue, Component } from 'vue-property-decorator'
import { cameraModule, victimModule } from '@/store'
import { CameraMap } from '@/store/modules/camera.types'

@Component
export default class VictimConfig extends Vue {
  cameras = cameraModule.camerasForSelect

  get camera(): string {
    return victimModule.camera
  }

  set camera(value) {
    victimModule.setCamera(value)
  }
}
</script>

<style lang="scss" scoped></style>
