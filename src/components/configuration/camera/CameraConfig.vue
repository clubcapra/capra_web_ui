<template>
  <b-section>
    <b-title>Camera</b-title>
    <hr />
    <diV class="cameras">
      <div v-for="(camera, key) in cameras" :key="key">
        <camera-card class="camera" :title="key" :camera-name="key" />
      </div>
    </diV>
  </b-section>
</template>

<script lang="ts">
import { Vue, Component } from 'vue-property-decorator'

import CameraCard from './CameraCard.vue'

import { cameraModule } from '@/store'

@Component({ components: { CameraCard } })
export default class CameraConfig extends Vue {
  get cameras() {
    return cameraModule.cameras
  }
}
</script>

<style lang="scss" scoped>
.cameras {
  display: grid;
  grid-template-columns: 1fr 1fr 1fr;
  grid-gap: 5px;
}
</style>
