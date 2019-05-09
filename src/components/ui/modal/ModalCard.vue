<template>
  <b-modal :is-active="visible">
    <b-modal-background />
    <b-modal-card>
      <b-modal-card-head>
        <b-modal-card-title>{{ title }}</b-modal-card-title>
        <button class="delete" aria-label="close" @click="close" />
      </b-modal-card-head>
      <b-modal-card-body>
        <slot />
      </b-modal-card-body>
      <b-modal-card-foot>
        <b-button is-success @click="ok">Ok</b-button>
        <b-button is-danger @click="cancel">Cancel</b-button>
      </b-modal-card-foot>
    </b-modal-card>
  </b-modal>
  <!-- <base-modal :visible="visible" @close="close">
    <card :title="title">
      <template v-if="closeable" #header-icon class="test">
        <b-button is-danger @click="close">
          <b-icon>
            <font-awesome-icon icon="times" />
          </b-icon>
        </b-button>
      </template>
      <template #default>
        <slot />
      </template>
      <template #footer>
        <button
          class="button card-footer-item is-success is-fullsize"
          @click="ok"
        >
          Ok
        </button>
        <button
          class="button card-footer-item is-danger is-fullsize"
          @click="cancel"
        >
          Cancel
        </button>
      </template>
    </card>
  </base-modal> -->
</template>

<script lang="ts">
import { Vue, Component, Prop } from 'vue-property-decorator'
import BaseModal from '@/components/ui/modal/BaseModal.vue'
import Card from '@/components/ui/card/Card.vue'

@Component({ components: { BaseModal, Card } })
export default class ModalCard extends Vue {
  @Prop({ default: false })
  readonly visible!: boolean

  @Prop({ default: false })
  readonly closeable!: boolean

  @Prop()
  readonly title?: string

  @Prop({ default: () => {} })
  readonly onOk!: Function

  @Prop({ default: () => {} })
  readonly onCancel!: Function

  @Prop({ default: () => {} })
  readonly onClose!: Function

  ok() {
    console.log('ok')
    this.onOk()
    this.close()
  }

  cancel() {
    this.onCancel()
    this.close()
  }

  close() {
    this.onClose()
    this.$emit('close')
  }
}
</script>

<style lang="scss" scoped></style>
