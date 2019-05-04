<template>
  <div class="victim">
    <Camera :type="camera.type" :topic="camera.topic" />
    <div class="sensors">
      <div>
        <h2 class="header">QR</h2>
        <br />
        <div v-for="qr in QRcodes" :key="qr.id">
          <p>{{ qr.number }} - {{ qr.text }}</p>
        </div>
      </div>
      <br />
      <div>
        <h2 class="header">Landolt</h2>
        <br />
        <div v-for="landolt in landolts" :key="landolt.id">
          <p>
            <font-awesome-icon
              icon="circle"
              :style="{ color: landolt.color }"
            />
            - {{ landolt.angle }}
          </p>
        </div>
      </div>
    </div>
  </div>
</template>

<script lang="ts">
import { Vue, Component } from 'vue-property-decorator'

import Camera from '@/components/Camera.vue'

import CameraModule from '@/store/modules/camera'

@Component({ components: { Camera } })
export default class Victim extends Vue {
  QRcodes = [
    {
      id: '0',
      number: '0',
      text: 'test value',
    },
    {
      id: '1',
      number: '1',
      text: 'test value',
    },
  ]

  landolts = [
    { id: '0', color: 'red', angle: '90' },
    { id: '1', color: 'green', angle: '45' },
  ]

  get camera() {
    return CameraModule.cameras['camera1']
  }
}
</script>

<style lang="scss">
.victim {
  display: grid;
  grid-template-columns: 60% 40%;

  .sensors {
    box-shadow: inset 0 0 0 1px #000000;
    .header {
      // text-decoration: underline;
      font-weight: bold;
    }
  }
}
</style>
