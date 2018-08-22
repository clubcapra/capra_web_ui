<template lang="pug">
  img(:src="stream")
</template>

<script>
import { mapState } from 'vuex'

export default {
  name: 'FrontCameraRGB',
  computed: mapState({
    stream: state => (state.ros.connected ? state.ros.camera.front.rgb : '')
  })
}
</script>
