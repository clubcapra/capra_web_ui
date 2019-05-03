<template>
  <div class="camera">
    <div v-if="connected">
      <img v-if="type === 'mjpeg'" :src="stream" />
      <video v-else-if="type === 'vp8'" :src="stream" autoplay preload="none" />
      <div v-else class="no-video"><p>invalid type</p></div>
    </div>
    <div v-else>
      <div class="no-video"><p>no video</p></div>
    </div>
  </div>
</template>

<script lang="ts">
import { Vue, Component, Prop } from 'vue-property-decorator'

import CameraModule from '@/store/modules/camera'
import RosModule from '@/store/modules/ros'

@Component
export default class Camera extends Vue {
  @Prop({ default: 'mjpeg' })
  readonly type!: string

  @Prop({ default: '', required: true })
  readonly topic!: string

  get connected() {
    return RosModule.connected
  }

  get stream() {
    const url = `http://${CameraModule.videoServerIP}/stream
    ?topic=${this.topic}
    &type=${this.type}
    `

    return this.connected ? url : ''
  }
}
</script>

<style lang="scss" scoped>
.camera {
  height: 100%;
  display: grid;
  min-height: 0px;
  margin: auto;
  width: 100%;

  img {
    height: 100%;
  }

  .no-video {
    display: grid;
    height: 100%;

    p {
      margin: auto;
    }
  }
}
</style>
