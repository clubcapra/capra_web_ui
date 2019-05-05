<template>
  <card :title="title">
    <b-field>
      <b-label>Topic</b-label> <b-input v-model="topic" is-small />
    </b-field>

    <b-field>
      <b-label>Type</b-label>
      <b-control>
        <b-select is-small>
          <select v-model="type">
            <option
              v-for="key in types"
              :key="key.value"
              :value="key.value"
              :disabled="key.disabled"
              >{{ key.value }}</option
            >
          </select>
        </b-select>
      </b-control>
    </b-field>
  </card>
</template>

<script lang="ts">
import { Vue, Component, Prop } from 'vue-property-decorator'

import Card from '@/components/UI/Card/Card.vue'

import CameraModule from '@/store/modules/camera'
import { CameraType } from '@/store/modules/camera.types'

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
    return CameraModule.typesForSelect
  }

  get type() {
    return this.camera.type
  }

  set type(value: string) {
    CameraModule.setType({
      cameraName: this.cameraName,
      type: value as CameraType,
    })
  }
}
</script>
