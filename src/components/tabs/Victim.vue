<template>
  <div class="victim">
    <Camera :type="cameraMotion.type" :topic="cameraMotion.topic" />
    <Camera :type="cameraThermal.type" :topic="cameraThermal.topic" />
    <Camera :type="cameraQr.type" :topic="cameraQr.topic" />
    <Camera :type="cameraHazmat.type" :topic="cameraHazmat.topic" />

    <div class="sensors">
      <div>CO2:</div>
      <CO2Graph />
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

  get cameraMotion() {
    return cameraModule.cameras[victimModule.cameraMotion]
  }

  get cameraThermal() {
    return cameraModule.cameras[victimModule.cameraThermal]
  }

  get cameraQr() {
    return cameraModule.cameras[victimModule.cameraQr]
  }

  get cameraHazmat() {
    return cameraModule.cameras[victimModule.cameraHazmat]
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
