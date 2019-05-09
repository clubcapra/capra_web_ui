<template>
  <div class="e-stop-btn">
    <b-button is-danger @click="sendServiceStop">
      <p>
        <span>E</span>
        <br />
        <span>M</span>
        <br />
        <span>E</span>
        <br />
        <span>R</span>
        <br />
        <span>G</span>
        <br />
        <span>E</span>
        <br />
        <span>N</span>
        <br />
        <span>C</span>
        <br />
        <span>Y</span>
        <br />
        <span />
        <br />
        <span>S</span>
        <br />
        <span>T</span>
        <br />
        <span>O</span>
        <br />
        <span>P</span>
        <br />
      </p>
    </b-button>
    <modal-card
      :visible="isModalVisible"
      title="Warning!"
      closeable
      @close="onModalClose"
      >Robot is currently stopped. Do you want to restart it?</modal-card
    >
  </div>
</template>

<script lang="ts">
import { Vue, Component, Inject } from 'vue-property-decorator'
import RosClient from '@/utils/ros/RosClient'
import ModalCard from '@/components/ui/modal/ModalCard.vue'

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
  p {
    font-weight: bold;
  }
}
</style>
