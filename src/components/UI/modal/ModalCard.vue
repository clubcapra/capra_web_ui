<template>
  <base-modal :visible="visible" @close="close">
    <card :title="title">
      <template v-if="closeable" #header-icon :style="{ padding: 0 }">
        <b-button id="close-button" is-danger @click="close">
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
          @click="close"
        >
          Ok
        </button>
        <button
          class="button card-footer-item is-danger is-fullsize"
          @click="close"
        >
          Cancel
        </button>
      </template>
    </card>
  </base-modal>
</template>

<script lang="ts">
import { Vue, Component, Prop } from 'vue-property-decorator'
import BaseModal from './BaseModal.vue'
import Card from '@/components/UI/Card/Card.vue'

@Component({ components: { BaseModal, Card } })
export default class ModalCard extends Vue {
  @Prop({ default: false })
  readonly visible!: boolean

  @Prop({ default: false })
  readonly closeable!: boolean

  @Prop()
  readonly title?: string

  close() {
    this.$emit('close')
  }
}
</script>

<style lang="scss" scoped></style>
