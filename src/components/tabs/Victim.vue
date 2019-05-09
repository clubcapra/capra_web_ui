<template>
  <div class="victim">
    <Camera :type="camera.type" :topic="camera.topic" />
    <div class="sensors">
      <h2 class="header">QR</h2>
      <br />
      <div v-for="qr in QRcodes" :key="qr.id">
        <p>{{ qr.number }} - {{ qr.text }}</p>
      </div>
      <hr />
      <h2 class="header">Landolt</h2>
      <br />
      <div v-for="landolt in landolts" :key="landolt">
        <p>
          <font-awesome-icon icon="circle" />
          - {{ landolt }}
        </p>
      </div>
      <hr />
      <h2 class="header">Hazmat</h2>
      <br />
      <div v-for="hazmat in hazmats" :key="hazmat.Class">
        <p>- {{ hazmat.Class }} probability: {{ hazmat.probability }}</p>
      </div>
    </div>
  </div>
</template>

<script lang="ts">
import { Vue, Component, Inject } from 'vue-property-decorator'

import RosClient from '@/utils/ros/RosClient'
import { Camera } from '@/components'
import { cameraModule } from '@/store'

@Component({ components: { Camera } })
export default class Victim extends Vue {
  @Inject('rosClient') rosClient!: RosClient

  QRcodes = [
    {
      id: '0',
      number: '0',
      text: 'test value',
    },
  ]

  landolts: Array<number> = []

  hazmats: Array<{ Class: string; probability: number }> = []

  get camera() {
    return cameraModule.cameras['camera1']
  }

  mounted() {
    this.subscribeToTopics()
  }

  setHazmats(boundingBoxes: []) {
    this.hazmats = boundingBoxes
  }

  setLandolts(params: { angles: number[] }) {
    this.landolts = params.angles
  }

  setQR(QRcodes: []) {
    this.QRcodes = QRcodes
  }

  subscribeToTopics() {
    this.rosClient.subscribe(
      {
        name: '/bounding_boxes',
        messageType: 'darknet_ros_msgs/BoundingBoxes',
      },
      this.setHazmats
    )

    this.rosClient.subscribe(
      {
        name: '/landolts',
        messageType: 'capra_landolt_msgs/Landolts',
      },
      this.setLandolts
    )
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
      font-weight: bold;
    }
  }
}
</style>
