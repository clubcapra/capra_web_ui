<template>
  <div class="e-stop-btn">
    <b-button class="is-fullsize" is-danger @click="sendServiceStop">
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
    <ok-cancel-modal
      :visible="isModalVisible"
      title="Warning!"
      ok-button-text="Yes"
      cancel-button-text="No"
      @close="onModalClose"
      @ok="onModalOk"
      @cancel="onModalCancel"
    >Robot is currently stopped. Do you want to restart it?</ok-cancel-modal>
  </div>
</template>

<script lang="ts">
import { Vue, Component, Inject } from 'vue-property-decorator'
import RosClient from '@/utils/ros/RosClient'
import { OkCancelModal } from '@/components/ui/modal'

@Component({ components: { OkCancelModal } })
export default class EStop extends Vue {
  isModalVisible = false

  sendServiceStop() {
    RosClient.callService({ name: 'takin_estop_disable', serviceType: '' }, '')
    this.isModalVisible = true
  }

  onModalClose() {
    this.isModalVisible = false
  }

  onModalOk() {
    RosClient.callService({ name: 'takin_estop_enable', serviceType: '' }, '')
    this.isModalVisible = false
  }

  onModalCancel() {
    console.log('cancel estop')
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
