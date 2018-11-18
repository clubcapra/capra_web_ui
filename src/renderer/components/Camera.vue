<script>
import { mapState } from 'vuex'
import { get as _get } from 'lodash'

export default {
  name: 'Camera',
  props: { path: { type: String, default: '' } },
  computed: {
    ...mapState('ros', { connected: state => state.connected }),
    stream() {
      return this.connected ? _get(this.cameras, this.path) : ''
    }
  },
  render() {
    const { connected, stream } = this
    return (
      <div class="camera">
        {connected ? (
          <div>
            {/* <video src={stream} autoplay preload="none" /> */}
            <img src={stream} />
          </div>
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
