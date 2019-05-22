<template>
  <base-modal :visible="visible" :closeable="closeable" @close="close">
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
          {{ okButtonText }}
        </button>
        <button
          class="button card-footer-item is-danger is-fullsize"
          @click="cancel"
        >
          {{ cancelButtonText }}
        </button>
      </template>
    </card>
  </base-modal>
</template>

<script lang="ts">
import { Vue, Component, Prop } from 'vue-property-decorator'
import BaseModal from '@/components/ui/modal/BaseModal.vue'
import Card from '@/components/ui/Card.vue'

@Component({ components: { BaseModal, Card } })
export default class OkCancelModal extends Vue {
  @Prop({ default: false })
  readonly visible!: boolean

  @Prop({ default: false })
  readonly closeable!: boolean

  @Prop()
  readonly title?: string

  @Prop({ default: () => {} })
  readonly onOk!: Function

  @Prop({ default: 'Ok' })
  readonly okButtonText!: String

  @Prop({ default: () => {} })
  readonly onCancel!: Function

  @Prop({ default: 'Cancel' })
  readonly cancelButtonText!: String

  @Prop({ default: () => {} })
  readonly onClose!: Function

  ok() {
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
