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

import Card from '@/components/ui/card/Card.vue'

import { cameraModule } from '@/store'
import { CameraType } from '@/store/modules/camera.types'

@Component({ components: { Card } })
export default class CameraCard extends Vue {
  @Prop({ type: String, default: '' }) title!: string
  @Prop({ type: String, default: '' }) cameraName!: string

  get camera() {
    return cameraModule.cameras[this.cameraName]
  }

  get topic() {
    return this.camera.topic
  }

  set topic(event: any) {
    cameraModule.setTopic({
      cameraName: this.cameraName,
      topic: event.target.value,
    })
  }

  get types() {
    return cameraModule.typesForSelect
  }

  get type() {
    return this.camera.type
  }

  set type(value: string) {
    cameraModule.setType({
      cameraName: this.cameraName,
      type: value as CameraType,
    })
  }
}
</script>
