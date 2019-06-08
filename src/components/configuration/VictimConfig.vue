<template>
  <b-section>
    <b-title>Victim</b-title>
    <hr />
    <h3>Camera Motion</h3>
    <b-select is-small>
      <select v-model="cameraMotion">
        <option v-for="c in cameras" :key="c.name" :value="c.name">{{
          c.name
        }}</option>
      </select>
    </b-select>
    <h3>Camera Thermal</h3>
    <b-select is-small>
      <select v-model="cameraThermal">
        <option v-for="c in cameras" :key="c.name" :value="c.name">{{
          c.name
        }}</option>
      </select>
    </b-select>
    <h3>Camera Qr</h3>
    <b-select is-small>
      <select v-model="cameraQr">
        <option v-for="c in cameras" :key="c.name" :value="c.name">{{
          c.name
        }}</option>
      </select>
    </b-select>
    <h3>Camera Hazmat</h3>
    <b-select is-small>
      <select v-model="cameraHazmat">
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

  get cameraMotion(): string {
    return victimModule.cameraMotion
  }

  set cameraMotion(value) {
    victimModule.setCameraMotion(value)
  }

  get cameraThermal(): string {
    return victimModule.cameraThermal
  }

  set cameraThermal(value) {
    victimModule.setCameraThermal(value)
  }

  get cameraQr(): string {
    return victimModule.cameraQr
  }

  set cameraQr(value) {
    victimModule.setCameraQr(value)
  }

  get cameraHazmat(): string {
    return victimModule.cameraHazmat
  }

  set cameraHazmat(value) {
    victimModule.setCameraHazmat(value)
  }
}
</script>

<style lang="scss" scoped></style>
