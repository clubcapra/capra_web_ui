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
        <slot name="footer" />
      </b-modal-card-foot>
    </b-modal-card>
  </b-modal>
</template>

<script lang="ts">
import { Vue, Component, Prop } from 'vue-property-decorator'
import BaseModal from '@/components/ui/modal/BaseModal.vue'
import Card from '@/components/ui/Card.vue'

@Component({ components: { BaseModal, Card } })
export default class ModalCard extends Vue {
  @Prop({ default: false })
  readonly visible!: boolean

  @Prop({ default: false })
  readonly closeable!: boolean

  @Prop()
  readonly title?: string

  @Prop({ default: () => {} })
  readonly onClose!: Function

  close() {
    this.onClose()
    this.$emit('close')
  }
}
</script>

<style lang="scss" scoped></style>
