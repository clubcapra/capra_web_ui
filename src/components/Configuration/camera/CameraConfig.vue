<template>
  <b-section>
    <b-title>Camera</b-title>
    <hr />

    <div
      v-for="(camera, key) in cameras"
      :key="key"
      style="width: 550px; padding:12px"
    >
      <camera-card class="camera" :title="key" :camera-name="key" />
    </div>
  </b-section>
</template>

<script lang="ts">
import { Vue, Component } from 'vue-property-decorator'

import CameraCard from './CameraCard.vue'

import CameraModule from '@/store/modules/camera'

@Component({ components: { CameraCard } })
export default class CameraConfig extends Vue {
  get cameras() {
    return CameraModule.cameras
  }
}
</script>

<style lang="scss" scoped></style>
