<template>
  <div class="victim">
    <Camera :type="camera.type" :topic="camera.topic" />
    <Camera :type="camera.type" :topic="camera.topic" />
    <Camera :type="camera.type" :topic="camera.topic" />
    <Camera :type="camera.type" :topic="camera.topic" />

    <div class="sensors">
      <div>CO2:</div>
      <CO2Graph />
      <!--<h2 class="header">QR</h2>
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
      </div>-->
    </div>
  </div>
</template>

<script lang="ts">
import { Vue, Component, Inject } from 'vue-property-decorator'

import RosClient from '@/utils/ros/RosClient'
import { Camera, CO2Graph } from '@/components'
import { cameraModule, victimModule } from '@/store'
import { TopicOptions } from '../../utils/ros/types'

@Component({ components: { Camera, CO2Graph } })
export default class Victim extends Vue {
  QRcodes = [
    {
      id: '0',
      number: '0',
      text: 'test value',
    },
  ]

  landolts: Array<number> = []
  hazmats: Array<{ Class: string; probability: number }> = []

  private landoltTopic: TopicOptions = {
    name: '/landolts',
    messageType: 'capra_landolt_msgs/Landolts',
  }

  private boundingBoxTopic: TopicOptions = {
    name: '/bounding_boxes',
    messageType: 'darknet_ros_msgs/BoundingBoxes',
  }

  get camera() {
    return cameraModule.cameras[victimModule.camera]
  }

  mounted() {
    RosClient.subscribe(this.boundingBoxTopic, this.setHazmats)
    RosClient.subscribe(this.landoltTopic, this.setLandolts)
  }

  beforeDestroy() {
    RosClient.unsubscribe(this.boundingBoxTopic)
    RosClient.unsubscribe(this.landoltTopic)
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
}
</script>

<style lang="scss">
.victim {
  display: grid;
  grid-template-columns: 50% 50%;
  grid-template-rows: 40% 40% 20%;

  .sensors {
    //grid-area: sensors;
    grid-column: 1 / span 2;
    box-shadow: inset 0 0 0 1px #000000;
    .header {
      font-weight: bold;
    }
  }
}
</style>
