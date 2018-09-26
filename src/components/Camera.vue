<template>
  <div class="camera">
    <div v-if="connected">
      <video autoplay>
        <source
          :src="stream"
          type="video/webm">
      </video>
    </div>
    <div
      v-else
      class="no-video">
      <p>no video</p>
    </div>
  </div>
</template>

<script>
import { mapGetters, mapState } from 'vuex'
import _ from 'lodash'

export default {
  name: 'Camera',
  props: { path: { type: String, default: '' } },
  computed: {
    ...mapGetters('ros', { cameras: 'cameras' }),
    ...mapState('ros', { connected: state => state.connected }),
    stream() {
      return this.connected ? _.get(this.cameras, this.path) : ''
    }
  }
}
</script>

<style lang="stylus" scoped>
.camera
  height 100%
  display grid
  min-width 0

  .no-video
    margin auto
</style>
