<template>
  <card :title="title">
    <div class="field">
      <label>Topic</label> <b-input v-model="topic" is-small />
    </div>

    <div class="field">
      <label>Type</label>
      <b-control>
        <b-select is-small>
          <select v-model="type">
            <option v-for="key in types" :key="key" :value="key">{{
              key
            }}</option>
          </select>
        </b-select>
      </b-control>
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

  get types() {
    return ['mjpeg', 'vp8']
  }

  get type() {
    return this.camera.type
  }

  set type(value: any) {
    CameraModule.setType({
      cameraName: this.cameraName,
      type: value,
    })
  }
}
</script>
