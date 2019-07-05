<template>
  <div v-if="visible" class="wrapper">
    <b-modal :is-active="visible">
      <b-modal-background />
      <b-modal-content>
        <slot />
      </b-modal-content>
      <b-modal-close
        v-if="closeable"
        is-large
        aria-label="close"
        @click="$emit('close')"
      />
    </b-modal>
  </div>
</template>

<script lang="ts">
import { Vue, Component, Prop } from 'vue-property-decorator'

@Component
export default class BaseModal extends Vue {
  @Prop({ default: false })
  readonly visible!: boolean

  @Prop({ default: false })
  readonly closeable!: boolean
}
</script>
