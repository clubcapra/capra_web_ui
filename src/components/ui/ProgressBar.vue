<template>
  <div class="wrapper" :style="wrapperStyle">
    <div class="progress-bar" :style="barStyle" />
  </div>
</template>

<script lang="ts">
import { Vue, Component, Prop } from 'vue-property-decorator'
import { ColorRGB, ColorHSL } from '@/utils/colors/types'
import { interpolateRGB, hexToRgb } from '@/utils/colors'
import _ from 'lodash-es'

@Component
export default class ProgressBar extends Vue {
  @Prop({ required: true, default: 1 })
  readonly value!: number

  @Prop({ default: false })
  readonly reverse!: boolean

  @Prop({ default: false })
  readonly vertical!: boolean

  @Prop({ default: false })
  readonly fillParent!: boolean

  @Prop({ default: 100 })
  readonly maxWidth!: number

  @Prop({ default: 10 })
  readonly height!: number

  get color() {
    const color = interpolateRGB(
      hexToRgb('#00ff00'), // green
      hexToRgb('#ff0000'), // red
      this.value
    )
    return `rgb(${color.r}, ${color.g}, ${color.b})`
  }

  get barStyle() {
    const percent = `${this.value * 100}%`

    return {
      background: this.color,
      width: this.vertical ? '100%' : percent,
      height: this.vertical ? percent : '100%',
    }
  }

  get wrapperStyle() {
    return {
      width: this.fillParent ? '100%' : `${this.maxWidth}px`,
      height: this.fillParent ? '100%' : `${this.height}px`,
      alignItems: this.vertical && this.reverse ? 'start' : 'end',
      justifyItems: this.reverse ? 'end' : 'start',
    }
  }
}
</script>

<style lang="scss" scoped>
.wrapper {
  display: grid;
  .progress-bar {
    display: grid;
  }
}
</style>
