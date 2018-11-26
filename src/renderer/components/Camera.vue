<template>
  <div>
    <div v-if="connected">
      <img v-if="type === 'mjpeg'" :src="stream" />
      <video v-else-if="type === 'vp8'" :src="stream" autoplay preload="none" />
      <div v-else class="no-video"><p>invalid type</p></div>
    </div>
    <div v-else>
      <div class="no-video"><p>no video</p></div>
    </div>
  </div>
</template>

<script>
import { mapState } from 'vuex'

export default {
  name: 'Camera',
  props: {
    type: { type: String, default: 'mjpeg' },
    topic: { type: String, required: true, default: '' }
  },
  computed: {
    ...mapState('ros', {
      connected: state => state.connected
    }),
    ...mapState('camera', {
      video_server_ip: state => state.video_server_ip
    }),
    stream() {
      return this.connected
        ? `http://${this.video_server_ip}/stream?topic=${this.topic}&type=${
            this.type
          }`
        : ''
    }
  }
  // render() {
  //   const { connected, stream, type } = this

  //   const Viewer = () => {
  //     console.log(type, stream)
  //     if (type === 'mjpeg') {
  //       console.log('mjpeg img')
  //       return <img src={stream} />
  //     } else if (type === 'vp8') {
  //       return <video src={stream} autoplay preload="none" />
  //     } else {
  //       return (
  //         <div class="no-video">
  //           <p>invalid type</p>
  //         </div>
  //       )
  //     }
  //   }

  //   return (
  //     <div class="camera">
  //       {connected ? (
  //         <Viewer />
  //       ) : (
  //         <div class="no-video">
  //           <p>no video</p>
  //         </div>
  //       )}
  //     </div>
  //   )
  // }
}
</script>

<style lang="scss" scoped>
.camera {
  //   height: 100%;
  display: grid;
  min-height: 0px;
  //   margin: auto;
  //   width: 100%;

  //   img {
  //     height: 100%;
  //   }

  .no-video {
    margin: auto;
  }
}
</style>
