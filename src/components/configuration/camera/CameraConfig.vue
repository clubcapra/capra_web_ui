<template>
  <b-section>
    <b-title>Camera</b-title>
    <hr />

    <input-with-button
      ref="camToAdd"
      v-model="cameraNameToAdd"
      label="Add Camera"
      button-text="Add"
      :class="`is-success`"
      @click="addCamera"
    />

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
import { InputWithButton } from '@/components/ui'

import { cameraModule } from '@/store'
import { CameraType } from '@/store/modules/camera.types'

@Component({ components: { CameraCard, InputWithButton } })
export default class CameraConfig extends Vue {
  cameraNameToAdd = ''

  get cameras() {
    return cameraModule.cameras
  }

  addCamera() {
    cameraModule.addCamera({
      cameraName: this.cameraNameToAdd,
      options: { type: CameraType.MJPEG, topic: '/' },
    })
    this.cameraNameToAdd = ''
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
