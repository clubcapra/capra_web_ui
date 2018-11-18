<template>
  <div class="camera">
    <div v-if="connected">
      <video :src="stream" autoplay preload="none" />
      <!-- <img :src="stream"> -->
    </div>
    <div v-else class="no-video"><p>no video</p></div>
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

<style lang="scss" scoped>
.camera {
  height: 100%;
  display: grid;
  min-height: 0;
  margin: auto;
  width: 100%;

  img {
    height: 100%;
  }

  .no-video {
    margin: auto;
  }
}
</style>
