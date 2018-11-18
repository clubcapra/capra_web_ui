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
      connected: state => state.connected,
      robotIP: state => state.robotIP
    }),
    stream() {
      return this.connected
        ? `http://${this.robotIP}/stream?topic=${this.topic}&type=${this.type}`
        : ''
    }
  },
  render() {
    const { connected, stream, type } = this

    const Viewer = () => {
      if (type === 'mjpeg') {
        return <img src={stream} />
      } else if (type === 'vp8') {
        return <video src={stream} autoplay preload="none" />
      } else {
        return (
          <div class="no-video">
            <p>invalid type</p>
          </div>
        )
      }
    }

    return (
      <div class="camera">
        {connected ? (
          <Viewer />
        ) : (
          <div class="no-video">
            <p>no video</p>
          </div>
        )}
      </div>
    )
  }
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
