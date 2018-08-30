<template lang="pug">
  img(:src="stream")
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
</style>
