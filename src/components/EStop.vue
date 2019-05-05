<template>
  <div class="e-stop-btn">
    <b-button is-small is-danger class="e-stop-btn" @click="sendServiceStop"
      ><b-icon>
        <font-awesome-icon icon="exclamation-triangle" />
      </b-icon>
      <span>Stop!</span>
    </b-button>
    <modal-card
      :visible="isModalVisible"
      title="Warning!"
      closeable
      @close="onModalClose"
    >
      Robot is currently stopped. Do you want to restart it?
    </modal-card>
  </div>
</template>

<script lang="ts">
import { Vue, Component, Inject } from 'vue-property-decorator'
import RosClient from '../utils/ros/RosClient'

import ModalCard from '@/components/UI/modal/ModalCard.vue'

@Component({ components: { ModalCard } })
export default class EStop extends Vue {
  @Inject() rosClient!: RosClient

  isModalVisible = false

  sendServiceStop() {
    this.rosClient.callService({ name: 'takin_estop', serviceType: '' }, '')
    this.isModalVisible = true
  }

  onModalClose() {
    this.isModalVisible = false
  }
}
</script>

<style lang="scss" scoped>
.e-stop-btn {
  height: 100%;
  width: 100%;
}
</style>
