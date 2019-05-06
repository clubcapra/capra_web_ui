<script lang="tsx">
import { Vue, Component, Prop } from 'vue-property-decorator'

import CameraModule from '@/store/modules/camera'
import { CameraType } from '@/store/modules/camera.types.ts'
import RosModule from '@/store/modules/ros'
import { VNode } from 'vue'

@Component
export default class Camera extends Vue {
  @Prop({ type: String, default: CameraType.MJPEG })
  readonly type!: CameraType

  @Prop({ default: '', required: true })
  readonly topic!: string

  get connected() {
    return RosModule.connected
  }

  get stream() {
    const url = `http://${CameraModule.videoServerIP}/stream
    ?topic=${this.topic}
    &type=${this.type}
    `

    return this.connected ? url : ''
  }

  render(): VNode {
    const NoVideo = (context: any) => (
      <div class="no-video">
        <p>{context.props.text}</p>
      </div>
    )

    const Camera = () => {
      switch (this.type) {
        case CameraType.MJPEG:
          return <img src={this.stream} />
        case CameraType.VP8:
          return <video src={this.stream} autoplay preload="none" />
        default:
          return <NoVideo text="invalid type" />
      }
    }

    return (
      <div class="camera">
        {this.connected ? <Camera /> : <NoVideo text="no video" />}
      </div>
    )
  }
}
</script>

<style lang="scss" scoped>
.camera {
  height: 100%;
  display: grid;
  min-height: 0px;
  margin: auto;
  width: 100%;

  img {
    height: 100%;
  }

  .no-video {
    display: grid;
    height: 100%;

    p {
      margin: auto;
    }
  }
}
</style>
