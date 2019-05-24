<template>
  <b-section>
    <b-title>Victim</b-title>
    <hr />
    <h3>Camera</h3>
    <select v-model="camera">
      <option v-for="c in cameras" :key="c.name" :value="c.name">{{
        c.name
      }}</option>
    </select>
  </b-section>
</template>

<script lang="ts">
import { Vue, Component } from 'vue-property-decorator'
import { cameraModule, victimModule } from '@/store'
import { CameraMap } from '@/store/modules/camera.types'

@Component
export default class VictimConfig extends Vue {
  cameras = cameraModule.camerasForSelect
  camera = victimModule.camera
}
</script>

<style lang="scss" scoped></style>
