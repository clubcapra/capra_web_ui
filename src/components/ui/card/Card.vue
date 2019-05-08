<template>
  <b-card>
    <header v-if="isHeaderVisible" class="card-header">
      <b-card-header-title v-if="title">
        {{ title }}
      </b-card-header-title>
      <div v-if="hasHeaderIcon" class="card-header-icon">
        <slot name="header-icon" />
      </div>
    </header>
    <b-card-content>
      <slot />
    </b-card-content>
    <footer v-if="hasFooter" class="card-footer">
      <slot name="footer" />
    </footer>
  </b-card>
</template>

<script lang="ts">
import { Vue, Component, Prop } from 'vue-property-decorator'

@Component
export default class Card extends Vue {
  @Prop({ default: false })
  readonly closeable!: boolean

  @Prop()
  readonly title?: string

  get isHeaderVisible() {
    return this.title || this.hasHeaderIcon
  }

  get hasHeaderIcon() {
    return !!this.$slots['header-icon']
  }

  get hasFooter() {
    return !!this.$slots['footer']
  }
}
</script>
