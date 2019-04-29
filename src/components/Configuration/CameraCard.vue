<template>
  <card :title="title">
    <div class="field">
      <label>Topic</label> <b-input v-model="topic" is-small />
    </div>
    <div class="field">
      <label>Type</label> <b-input v-model="type" is-small />
    </div>
  </card>
</template>

<script lang="ts">
import { Vue, Component, Prop } from 'vue-property-decorator'

import Card from '@/components/UI/Card/Card.vue'

import CameraModule from '@/store/modules/camera'

@Component({ components: { Card } })
export default class CameraCard extends Vue {
  @Prop({ type: String, default: '' }) title!: string
  @Prop({ type: String, default: '' }) cameraName!: string

  get camera() {
    return CameraModule.cameras[this.cameraName]
  }

  get topic() {
    return this.camera.topic
  }

  set topic(event: any) {
    CameraModule.setTopic({
      cameraName: this.cameraName,
      topic: event.target.value,
    })
  }

  get type() {
    return this.camera.type
  }

  set type(event: any) {
    CameraModule.setType({
      cameraName: this.cameraName,
      type: event.target.value,
    })
  }
}
</script>
