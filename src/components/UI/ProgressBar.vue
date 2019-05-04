<template>
  <div
    :style="{ background: color, width: `${value * 100}%` }"
    class="default-bar-style"
  />
</template>

<script lang="ts">
import { Vue, Component, Prop } from 'vue-property-decorator'

@Component
export default class ProgressBar extends Vue {
  /**
   * value should be between 0 and 1
   */
  @Prop({ default: 1 })
  readonly value!: number

  get color() {
    let red,
      green = 0

    const limit = 0.5

    const formatValue = (value: number) => Math.round((value / limit) * 255)

    if (this.value >= limit) {
      green = 255 - formatValue(this.value - limit)
      red = 255
    } else {
      green = 255
      red = formatValue(this.value)
    }

    return `rgb(${red}, ${green}, 0)`
  }

  get barStyle() {
    return {
      background: this.color,
      width: `${this.value * 100}%`,
    }
  }
}
</script>

<style lang="scss" scoped>
.default-bar-style {
  display: inline-block;
  height: 10px;
}
</style>
